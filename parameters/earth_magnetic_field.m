function [B_ned_uT, info] = earth_magnetic_field(lla, when)
%EARTH_MAGNETIC_FIELD Earth's magnetic field at a geodetic position, from the World Magnetic Model.
%   [B_ned_uT, info] = earth_magnetic_field([lat_deg lon_deg alt_m])
%   [B_ned_uT, info] = earth_magnetic_field([lat_deg lon_deg alt_m], datetime)
%
%   B_ned_uT : field vector in NED [uT] (north, east, down), as the IMU block expects
%   info     : declination/inclination [deg], strength [uT], model and decimal year used
%
%   Uses wrldmagm (Aerospace Toolbox, newest WMM epoch available), falling back to igrfmagm.
%   Valid at any position on Earth, so the simulated magnetometer always agrees with the
%   declination PX4's EKF looks up for the simulated GPS position.
if nargin < 2 || isempty(when), when = datetime('now'); end
lat = lla(1); lon = lla(2); alt = lla(3);
dy = decyear(when);
if exist('wrldmagm', 'file')
    model = '';
    for epoch = ["2025", "2020", "2015v2"]
        try
            XYZ = wrldmagm(alt, lat, lon, dy, char(epoch));
            model = "WMM" + epoch; break
        catch
        end
    end
    if isempty(model)
        error('earth_magnetic_field:noEpoch', 'No WMM epoch in wrldmagm covers %.2f.', dy);
    end
elseif exist('igrfmagm', 'file')
    XYZ = igrfmagm(alt, lat, lon, dy); model = "IGRF";
else
    error('earth_magnetic_field:noModel', ...
        'Needs the Aerospace Toolbox (wrldmagm or igrfmagm) to compute the magnetic field.');
end
B_ned_uT = XYZ(:).' / 1000;   % nT -> uT
H = hypot(B_ned_uT(1), B_ned_uT(2));
info = struct('Model', model, 'DecimalYear', dy, ...
    'Declination_deg', atan2d(B_ned_uT(2), B_ned_uT(1)), ...
    'Inclination_deg', atan2d(B_ned_uT(3), H), ...
    'Strength_uT', norm(B_ned_uT));
end
