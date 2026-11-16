function [stages,mode,limit]=px4_tuning_menu()
%PX4_TUNING_MENU Numbered Command Window menu that selects what run_px4_tuning tunes.
%   [stages,mode,limit] = px4_tuning_menu() asks for a selection (works from Main.mlx too) and returns
%   the stage keys, the session mode ("verify", "tune", "validate", or "guide"/"rmse" for the
%   multi-axis tuner) and the trial limit. Numbers 1-16 map to single loops through
%   px4_tuning_selection; 17 and 18 start run_px4_guide_tuning with the guide or RMSE method.
while true
    fprintf(['\nPX4 TUNING SESSION - 10 target steps/trial (minimum 5), 10 m target, filters fixed\n' ...
        ' Single loops: Default-only joint search; fixed rate K; hand gains are comparison only.\n 1. All supported loops\n 2. Rates (roll, pitch, yaw)\n' ...
        ' 3. Roll rate\n 4. Pitch rate\n 5. Yaw rate\n' ...
        ' 6. Angles (roll, pitch, yaw)\n 7. Roll angle\n 8. Pitch angle\n' ...
        ' 9. Velocity (horizontal, vertical)\n10. Horizontal velocity\n11. Vertical velocity\n' ...
        '12. Position (horizontal, vertical)\n13. Horizontal position\n14. Vertical position\n' ...
        '15. Separate acceleration loop (not exposed)\n16. Yaw angle\n' ...
        ' All loop groups, multi-axis flights:\n' ...
        '17. GUIDE-BASED AUTOMATIC TUNING (PX4 tuning-guide rules)\n' ...
        '18. RMSE-BASED AUTOMATIC TUNING (RMSE score search + robustness probe; recommended)\n 0. Cancel\n']);
    number=str2double(input('Type selection number: ','s'));
    if number==17,stages="guide";mode="guide";limit=0;return;end
    if number==18,stages="rmse";mode="rmse";limit=0;return;end
    try,stages=px4_tuning_selection(number);catch ME,fprintf('%s\n',ME.message);continue;end
    mode="verify";limit=3;
    if isempty(stages),return;end
    fprintf('1. Short verification (one loop, at most 3 trials)\n2. Joint tuning (stagnation checked after complete gradient cycles)\n3. Validate selected loops (one baseline each)\n');
    choice=str2double(input('Type session mode [1]: ','s'));if isnan(choice),choice=1;end
    if choice==1
        if numel(stages)>1||stages=="all"
            fprintf('Short verification requires one individual loop. Please select it.\n');continue;
        end
        return;
    elseif choice==3
        mode="validate";limit=1;return;
    elseif choice==2
        mode="tune";
        entry=input('Maximum trials per stage [30]: ','s');
        if isempty(entry),limit=30;else,limit=str2double(entry);end
        if ~isfinite(limit)||limit<1||fix(limit)~=limit,fprintf('Enter a positive whole-number trial limit.\n');continue;end
        return;
    else
        fprintf('Choose session mode 1, 2 or 3.\n');
    end
end
end
