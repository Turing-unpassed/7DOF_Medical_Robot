function Validate_Jacobian(link, q)
    % q in degrees
    Jb = Jocobian_Build(link, q);
    T0 = Forward_kinematics(link, q, 0); % must return 4x4 transform
    p0 = T0(1:3,4);
    R0 = T0(1:3,1:3);

    fprintf('i | posErr_norm(mm) | angErr_norm(rad)\n');
    for i=1:7
        dq_deg = zeros(1,7); dq_deg(i) = 1e-5; % try 1e-4 deg
        q2 = q + dq_deg;
        T1 = Forward_kinematics(link, q2, 0);
        p1 = T1(1:3,4);
        R1 = T1(1:3,1:3);

        % numeric displacement
        delta_p_num = p1 - p0;
        % predicted (use base-frame Jb; convert dq to rad)
        delta_p_pred = Jb(1:3,i) * (dq_deg(i)*pi/180);

        posErr = norm(delta_p_num - delta_p_pred);

        % rotation numeric: R_err = R1 * R0'
        R_err = R1 * R0';
        tr = (trace(R_err)-1)/2;
        tr = max(-1,min(1,tr));
        angle = acos(tr);
        if abs(angle) < 1e-12
            rotVec_num = [0;0;0];
        else
            axis = (1/(2*sin(angle))) * [R_err(3,2)-R_err(2,3); R_err(1,3)-R_err(3,1); R_err(2,1)-R_err(1,2)];
            rotVec_num = axis * angle; % rad
        end
        rot_pred = Jb(4:6,i) * (dq_deg(i)*pi/180);
        angErr = norm(rotVec_num - rot_pred);

        fprintf('%d | %e | %e\n', i, posErr, angErr);
    end
end