function [pkt, rd] = cameraReaderHRC(rd, port)
% Receives the messages of the camera program (mental_fatigue_algorithm/main.py,
% section pomdp_bridge of its config.json) over UDP and keeps the NEWEST one.
%
% [~, rd] = cameraReaderHRC([], port) open the receiver (port 5005 by default)
% [pkt, rd] = cameraReaderHRC(rd)    newest message (struct from jsondecode, [] if
%                                    nothing was received yet) and the reader
% rd.lastRx : reception time of pkt [s, clock of this reader: toc(rd.t0)]
% rd.count  : messages received, rd.bad : messages that could not be decoded
% The camera program sends ~15 messages per second; the live loop reads every
% 0.5 s, drops the older ones and uses the newest. A camera program that stops
% sending shows up as old geometry (geometryObservationHRC: stale -> not verified).
% Needs MATLAB R2020b or newer (udpport).
if isempty(rd)
    if nargin < 2 || isempty(port), port = 5005; end
    rd.u = udpport("datagram", "IPV4", "LocalPort", port, "Timeout", 0.05);
    rd.t0 = tic;
    rd.last = [];
    rd.lastRx = -Inf;
    rd.count = 0;
    rd.bad = 0;
    rd.port = port;
    pkt = [];
    return
end
n = rd.u.NumDatagramsAvailable;
if n > 0
    d = read(rd.u, n, "uint8");
    rd.count = rd.count + n;
    for i = n:-1:1                                % newest message that can be decoded
        try
            p = jsondecode(char(d(i).Data));
            rd.last = p;
            rd.lastRx = toc(rd.t0);
            break
        catch
            rd.bad = rd.bad + 1;
        end
    end
end
pkt = rd.last;
end
