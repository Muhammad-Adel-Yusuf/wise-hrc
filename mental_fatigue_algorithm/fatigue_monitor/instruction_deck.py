"""PowerPoint instruction deck: slide number = instruction number (for Algorithm 1).

The worker presses U in the camera window, picks a .pptx file, and then steps through it with the arrow keys.
The slides are shown in their own "Instructions" window and the current slide number is the current instruction
number fed to the task-difficulty algorithm (main.py: InstructionFeed).

Slide images (best available method, tried in this order):
  1. Microsoft PowerPoint itself (Windows, needs `pip install pywin32`) - exact rendering;
  2. LibreOffice (`soffice`) -> PDF -> PyMuPDF (`pip install pymupdf`) or poppler `pdftoppm`;
  3. plain text card built from the slide text (no extra software; slide count and numbers are still exact).
The slide count and slide texts are always read directly from the .pptx (zip/XML), so the numbers never depend
on the renderer.
"""
from __future__ import annotations

import glob
import logging
import re
import shutil
import subprocess
import tempfile
import textwrap
import threading
import zipfile
from pathlib import Path
from typing import Callable, List, Optional
from xml.etree import ElementTree as ET

import cv2
import numpy as np

log = logging.getLogger("fatigue.deck")

_NS = {"p": "http://schemas.openxmlformats.org/presentationml/2006/main",
       "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
       "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
       "rel": "http://schemas.openxmlformats.org/package/2006/relationships"}
_RID = "{%s}id" % _NS["r"]


def read_pptx_texts(path: str) -> List[str]:
    """Text of every slide in presentation order (one string per slide). Raises ValueError for a bad file."""
    try:
        with zipfile.ZipFile(path) as z:
            pres = ET.fromstring(z.read("ppt/presentation.xml"))
            rels = ET.fromstring(z.read("ppt/_rels/presentation.xml.rels"))
            target = {r.get("Id"): r.get("Target") for r in rels.findall("rel:Relationship", _NS)}
            texts: List[str] = []
            for sid in pres.findall("p:sldIdLst/p:sldId", _NS):
                tgt = target.get(sid.get(_RID), "")
                name = tgt.lstrip("/")
                name = name if name.startswith("ppt/") else "ppt/" + name
                root = ET.fromstring(z.read(name))
                paras = []
                for para in root.iter("{%s}p" % _NS["a"]):
                    t = "".join(x.text or "" for x in para.iter("{%s}t" % _NS["a"])).strip()
                    if t:
                        paras.append(t)
                texts.append("\n".join(paras))
            return texts
    except (zipfile.BadZipFile, KeyError, ET.ParseError, OSError) as exc:
        raise ValueError(f"not a readable .pptx file: {exc}") from exc


def _natural_key(p: str):
    return [int(x) if x.isdigit() else x.lower() for x in re.split(r"(\d+)", Path(p).name)]


def render_with_powerpoint(path: str, outdir: str, width: int) -> List[str]:
    import win32com.client                                   # type: ignore  # Windows + pywin32 only
    app = win32com.client.Dispatch("PowerPoint.Application")
    pres = app.Presentations.Open(str(Path(path).resolve()), True, False, False)   # ReadOnly, Untitled, WithWindow
    try:
        pres.Export(outdir, "PNG", width, int(width * 9 / 16))
    finally:
        pres.Close()
    return sorted(glob.glob(str(Path(outdir) / "*.PNG")) + glob.glob(str(Path(outdir) / "*.png")), key=_natural_key)


def render_with_libreoffice(path: str, outdir: str, width: int) -> List[str]:
    soffice = shutil.which("soffice") or shutil.which("libreoffice")
    if not soffice:
        for cand in (r"C:\Program Files\LibreOffice\program\soffice.exe", r"C:\Program Files (x86)\LibreOffice\program\soffice.exe"):
            if Path(cand).exists():
                soffice = cand
    if not soffice:
        raise RuntimeError("LibreOffice not found")
    subprocess.run([soffice, "--headless", "--convert-to", "pdf", "--outdir", outdir, str(path)],
                   check=True, timeout=180, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    pdfs = glob.glob(str(Path(outdir) / "*.pdf"))
    if not pdfs:
        raise RuntimeError("LibreOffice produced no PDF")
    pdf = pdfs[0]
    try:
        import fitz                                          # type: ignore  # PyMuPDF
        doc = fitz.open(pdf)
        out = []
        for i, page in enumerate(doc):
            zoom = width / page.rect.width
            f = str(Path(outdir) / f"page_{i + 1:04d}.png")
            page.get_pixmap(matrix=fitz.Matrix(zoom, zoom)).save(f)
            out.append(f)
        return out
    except ImportError:
        pass
    if not shutil.which("pdftoppm"):
        raise RuntimeError("need PyMuPDF (pip install pymupdf) or poppler (pdftoppm) to draw the PDF pages")
    subprocess.run(["pdftoppm", "-png", "-scale-to-x", str(width), "-scale-to-y", "-1", pdf, str(Path(outdir) / "page")],
                   check=True, timeout=180)
    return sorted(glob.glob(str(Path(outdir) / "page*.png")), key=_natural_key)


class InstructionDeck:
    """Holds the slides of one .pptx. load() returns at once; rendering runs in a background thread."""

    def __init__(self, render_width: int = 1280) -> None:
        self.render_width = render_width
        self.path: Optional[str] = None
        self.texts: List[str] = []
        self.images: List[Optional[np.ndarray]] = []
        self.status = "no slides loaded (press U)"
        self.renderer = "none"
        self._lock = threading.Lock()
        self._tmp: Optional[str] = None
        self._cache: dict = {}

    @property
    def count(self) -> int:
        return len(self.texts)

    @property
    def loaded(self) -> bool:
        return self.count > 0

    def load(self, path: str, on_ready: Optional[Callable[[int], None]] = None) -> int:
        """Read the slide list (fast, synchronous) and start rendering images in the background. Returns slide count."""
        texts = read_pptx_texts(path)
        if not texts:
            raise ValueError("the presentation has no slides")
        with self._lock:
            self.path, self.texts = str(path), texts
            self.images = [None] * len(texts)
            self._cache = {}
            self.renderer = "text"
            self.status = f"{Path(path).name}: {len(texts)} slides (rendering ...)"
        threading.Thread(target=self._render, args=(str(path),), daemon=True).start()
        if on_ready:
            on_ready(len(texts))
        return len(texts)

    def _render(self, path: str) -> None:
        tmp = tempfile.mkdtemp(prefix="deck_")
        self._tmp = tmp
        errors = []
        for name, fn in (("PowerPoint", render_with_powerpoint), ("LibreOffice", render_with_libreoffice)):
            try:
                files = fn(path, tmp, self.render_width)
                imgs = [cv2.imread(f) for f in files]
                if len(imgs) == self.count and all(i is not None for i in imgs):
                    with self._lock:
                        if self.path != path:
                            return
                        self.images, self.renderer = imgs, name
                        self._cache = {}
                        self.status = f"{Path(path).name}: {self.count} slides ({name} rendering)"
                    return
                errors.append(f"{name}: got {len(imgs)} images for {self.count} slides")
            except Exception as exc:                          # noqa: BLE001
                errors.append(f"{name}: {exc}")
        log.warning("Slide images not available (%s); showing slide text instead.", "; ".join(errors))
        with self._lock:
            self.status = f"{Path(path).name}: {self.count} slides (text view - install PowerPoint/pywin32 or LibreOffice+pymupdf for images)"

    # -- display -----------------------------------------------------------------------
    def frame(self, step: Optional[int], size=(960, 600)) -> np.ndarray:
        """BGR image for instruction number `step` (1-based slide number)."""
        w, h = size
        n = self.count
        s = 1 if step is None else int(min(max(step, 1), max(n, 1)))
        key = (s, self.renderer, w, h)
        with self._lock:
            hit = self._cache.get(key)
            if hit is not None:
                return hit
            img = self.images[s - 1] if 0 < s <= len(self.images) else None
            text = self.texts[s - 1] if 0 < s <= len(self.texts) else ""
            status = self.status
        canvas = np.full((h, w, 3), 245, np.uint8)
        bar = 34
        if img is not None:
            ih, iw = img.shape[:2]
            sc = min(w / iw, (h - bar) / ih)
            rs = cv2.resize(img, (max(1, int(iw * sc)), max(1, int(ih * sc))), interpolation=cv2.INTER_AREA)
            y0, x0 = bar + (h - bar - rs.shape[0]) // 2, (w - rs.shape[1]) // 2
            canvas[y0:y0 + rs.shape[0], x0:x0 + rs.shape[1]] = rs
        else:
            y = bar + 40
            for para in (text or "(empty slide)").split("\n"):
                for line in textwrap.wrap(para, 60) or [""]:
                    cv2.putText(canvas, line, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (30, 30, 30), 2, cv2.LINE_AA)
                    y += 34
                    if y > h - 20:
                        break
        cv2.rectangle(canvas, (0, 0), (w, bar), (60, 60, 60), -1)
        cv2.putText(canvas, f"INSTRUCTION {s} / {n}     Down/Right = next   Up/Left = previous   U = load file",
                    (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        with self._lock:
            self._cache[key] = canvas
        return canvas

    def close(self) -> None:
        if self._tmp:
            shutil.rmtree(self._tmp, ignore_errors=True)


def ask_for_pptx() -> Optional[str]:
    """File-open dialog (tkinter). Returns None if cancelled or tkinter is unavailable."""
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError:
        log.warning("tkinter is not available; start with --pptx path/to/file.pptx instead")
        return None
    root = tk.Tk()
    root.withdraw()
    try:
        root.attributes("-topmost", True)
        p = filedialog.askopenfilename(title="Select the PowerPoint instruction file",
                                       filetypes=[("PowerPoint", "*.pptx *.pptm *.ppsx"), ("All files", "*.*")])
    finally:
        root.destroy()
    return p or None
