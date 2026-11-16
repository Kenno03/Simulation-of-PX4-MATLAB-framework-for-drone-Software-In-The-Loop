function Plan = flight_plan(kind)
%FLIGHT_PLAN Mission references in PX4 NED metres and seconds.
% These settings define the commanded path, not the vehicle or controller.
% Kinds: "square" (Size = side), "circle" (Size = diameter), "figure8" (Size = length North-South;
% the width East-West is Size/2; one lap takes 4*(LegTime+DwellTime) like the circle).
arguments
    kind (1,1) string {mustBeMember(kind,["square","circle","figure8"])}
end
Plan.Kind = uint8(find(kind == ["square","circle","figure8"]));
Plan.Size = 1.0;                 % Square side; circle diameter; figure-8 length [m]
Plan.Altitude = 1.0;             % Height above local origin [m]
Plan.TakeoffStart = 20.0;
Plan.TakeoffTime = 8.0;
Plan.SettleTime = 4.0;
Plan.LegTime = 8.0;
Plan.DwellTime = 2.0;
Plan.FinalHoverTime = 10.0;
% true: the trajectory also supplies the analytic reference velocity (PX4 uses it as feedforward in its
% position loop; the custom LQR uses it as its velocity reference). false: position setpoints only.
Plan.VelocityFeedforward = false;
% true: on the circle and figure-8 the heading follows the direction of motion. false: fixed North.
Plan.HeadingAlongPath = false;
end
