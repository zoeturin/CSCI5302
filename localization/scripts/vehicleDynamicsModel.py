import numpy as np

class dynamicsModel:

    def __init__(self, c):
        # PARAMETERS FROM VEHICLE
        """
        it should be in the form of a DICTIONARY:
        cardict = {
            "engineMaxTorque": -
            "engineMaxPower": -
            "wheelRadius": -
            "wheelbase": -
            "wheeltrack": -
            "gearRatios": -
            "brakeCoefficient": -
            "centerOfMass": -
            "mass": -
            "momentOfInertia": -
        }
        """
        self.maxT = c["engineMaxTorque"];  # engineMaxTorque
        self.maxP = c["engineMaxPower"];  # engineMaxPower
        self.wheelRadius = c["wheelRadius"]; # radius of wheels
        self.wheelbase = c["wheelbase"]; #
        self.wheeltrack = c["wheeltrack"]; #
        self.gearRatios = c["gearRatios"]; #
        self.brakeCoefficient = c["brakeCoefficient"]; #
        self.centerOfMass = c["centerOfMass"]; #
        self.mass = c["mass"]; #
        self.momentOfInertia = c["momentOfInertia"]; # I11,I22,I33,I12,I13,I23
        self.momentOfInertiaAboutOrigin = self.momentOfInertia[0][1] \
            + self.mass * np.linalg.norm(self.centerOfMass)**2;

    def predictState(self,input,sp,dt):
        # input: vector: [throttle, brake, desired steering angle, desired gear]
        # sp: prior state (which we'll use to estimate)
        # dt: timestep
        # NOTE: INPUT IS THE INPUT DECIDED UPON AFTER ESTIMATION OF SP
        # (so in the previous timestep)
        # ASSUMPTIONS:
        #   no slipping
        #   +steerAngle = turning right
        #   change in steerAngle negligible
        #   right-handed coordinate system (turned right = -theta in state)

        [x,z,a,dx,dz,da] = sp; # x, z, theta, x', z', theta'
        [throttle, brake, desiredSteeringAngle, desiredGear] = input;
        gearRatio = self.gearRatios[desiredGear];


        # rotation matrix from original car orientation
        Q = np.array([ \
            [np.cos(a), 0, np.sin(a)], \
            [0, 1, 0], \
            [-np.sin(a), 0, np.cos(a)]]);

        # ABSTRACTION OF VEHICLE DYNAMICS (to useful quantities)
        position = np.array([x,0,z]); # position vector in global coordinates
        velocity = np.array([dx,0,dz]); # velocity vector in global coordinates

        currentSteeringAngle = 1; # TODO: GET THIS FROM CAR
        # psych let's use the desired one for now, for troubleshooting
        currentSteeringAngle = desiredSteeringAngle;

        # get current turn radius from true steering angle
        # (you could also do this from the state, but that might be slipping)

        # note: turnRadius is always >0; sign of steering angle determines direction
        turnRadius = self.wheeltrack / 2 + self.wheelbase / \
            (np.tan(currentSteeringAngle) - self.wheeltrack / (2 * self.wheelbase));
        turnRadius = np.abs(turnRadius);

        # INDIVIDUAL WHEEL VELOCITIES
        v = np.linalg.norm(velocity) * np.sign(np.dot(Q.dot(np.array([0,0,1])), velocity));
        rw = self.wheelRadius;
        R = turnRadius;
        # wheel: front-inside (of turn)
        rfi = np.sqrt((R - self.wheeltrack / 2)**2 + self.wheelbase**2);
        wfi = v * rfi / R / rw;
        #wfi = v * np.sqrt((R - self.wheeltrack / 2)**2 + R**2) / R / r;
        # wheel: front-outside
        rfo = np.sqrt((R + self.wheeltrack / 2)**2 + self.wheelbase**2);
        wfo = v * rfo / R / rw;
        # wheel: back-inside
        rbi = R - self.wheeltrack / 2;
        wbi = v * rbi / R / rw;
        # wheel: back-outside
        rbo = R + self.wheeltrack / 2;
        wbo = v * rbo / R / rw;

        if currentSteeringAngle > 0: # turning right
            wfr = wfi; wfl = wfo; # front-right and front-left
            wbr = wbi; wbl = wbo; # back-right and back-left
        else: # if turning left, the velocities are switched L/R
            wfr = wfo; wfl = wfi; # front-right and front-left
            wbr = wbo; wbl = wbi; # back-right and back-left

        # ENGINE TORQUE TO WHEEL TORQUES
        # estimate engine rpm from estimates of current wheel speeds:
        # use average of left and right rear wheels, and account for gearing.
        engineAngularVelocity = np.abs(gearRatio * (wbr + wbl) / 2);
        engineAngularVelocity = np.maximum(engineAngularVelocity,0.01);
        # Torque coming out of engine
        Temax = np.minimum(self.maxT, self.maxP / engineAngularVelocity);
        Te = Temax * throttle;
        # Torque going into the differential
        Td = gearRatio * Te;
        # Angular velocity of differential
        wd = engineAngularVelocity / gearRatio;
        # Torque variation between rear wheels
        Tv = Td * (wd - (wbr + wbl) / 2) / ((wbr - wbl) / 2);
        # (not actually finishing this calculation bc we don't need it)

        # ENGINE POWER
        Pe = Td * wd; # (just using the differential lol)

        # BRAKE POWER
        B = self.brakeCoefficient * brake; # brake damping (Nm / (rad/s) ???)
        Pb = -B * (wfr + wfl + wbr + wbl); # power by brakes on all wheels

        # NET POWER INTO SYSTEM
        Pnet = Pe + Pb;

        # COMPUTE NEW VELOCITY VIA KINETIC ENERGY
        I = self.momentOfInertiaAboutOrigin; # IYY
        m = self.mass;
        #LKE = 0.5 * m * v**2;   # linear kinetic energy
        #RKE = 0.5 * I * w**2;   # rotational kinetic energy
        W = Pnet * dt; # net work into the system
        vn = np.sign(v) * np.sqrt(v**2 + 2 * W / (m + I / (R**2))); # new velocity (speed actually)

        # GET PREDICTED STATE
        # moment of inertia about apex and angular acceleration (used for partial derivatives and force method)
        Iapex = self.momentOfInertiaAboutOrigin + self.mass * R**2;
        dda = (1/(Iapex*rw)) * \
            (R*Td - B*da*(2*R**2+rfi**2+rfo**2)/rw);

        mu = None;
        PREDICT_WITH_ENERGY = True;
        if PREDICT_WITH_ENERGY:
            # ENERGY METHODS
            # new rotational rate from vn
            da_n = -np.sign(currentSteeringAngle) * vn / R;
            # effective linear and rotational translations from old to new state
            linear_equivalent = dt * (vn + v)/2; # total travelled distance
            angular_equivalent = linear_equivalent / R; # total travelled rotation
            # translation along turn, converted into x, y, and z components in car's coordinates:
            dp_local = np.array([ \
                R * (1 - np.cos(angular_equivalent)) * -np.sign(currentSteeringAngle), \
                0, \
                R * np.sin(angular_equivalent)]);
            # translation along turn, but in global coordinates
            dp_global = Q.dot(dp_local);
            # new velocity in global coordinates
            vn_global = Q.dot(np.array([0,0,vn]));

            # create final, new state estimate:
            x_n = x + dp_global[0];
            z_n = z + dp_global[2];
            a_n = a + angular_equivalent;

            dx_n = vn_global[0];
            dz_n = vn_global[2];
            da_n = da_n;

            mu = np.array([x_n,z_n,a_n,dx_n,dz_n,da_n]);
        else:
            # FORCE BALANCE METHODS
            x_n = x + dt*dx;
            z_n = z + dt*dz;
            a_n = a + dt*da;
            dx_n = dx + dt*(R* np.cos(a)*dda - R*np.sin(a)*da**2);
            dz_n = dz + dt*(R*-np.sin(a)*dda - R*np.cos(a)*da**2);
            da_n = da + dt*dda;
            mu = np.array([x_n,z_n,a_n,dx_n,dz_n,da_n]);


        # PARTIAL DERIVATIVES FOR THE EKF

        dFdx = np.array([1,0,0,0,0,0]);
        dFdz = np.array([0,1,0,0,0,0]);
        dFda = np.array([0,0,2, \
            dt*(R*dda*-np.sin(a) - da**2 * R*np.cos(a)), \
            dt*(R*dda*-np.cos(a) + da**2 * R*np.sin(a)), \
            0]);
        dFddx= np.array([dt,0,0,1,0,0]);
        dFddz= np.array([0,dt,0,0,1,0]);
        dFdda= np.array([0,0,dt, \
            dt*( R*np.cos(a)*( (1/(Iapex*rw))*-B*(2*R**2+rfi**2+rfo**2)/rw ) \
            - 2*da*R*np.sin(a) ), \
            dt*(-R*np.sin(a)*( (1/(Iapex*rw))*-B*(2*R**2+rfi**2+rfo**2)/rw ) \
            - 2*da*R*np.cos(a) ), \
            dt*( (1/(Iapex*rw))*-B*(2*R**2+rfi**2+rfo**2)/rw ) ]);
        # throttle and brake:
        dFdp = np.array([0,0,0, \
            dt* R*np.cos(a)*(1/(Iapex*rw))*(R*Td), \
            dt*-R*np.sin(a)*(1/(Iapex*rw))*(R*Td), \
            dt* (1/(Iapex*rw))*(R*Td) ]);
        dFdb = np.array([0,0,0, \
            dt* R*np.cos(a)*(1/(Iapex*rw))*da*-self.brakeCoefficient*(2*R**2+rfi**2+rfo**2)/rw, \
            dt*-R*np.sin(a)*(1/(Iapex*rw))*da*-self.brakeCoefficient*(2*R**2+rfi**2+rfo**2)/rw ]);

        # combine into matrix:
        F = np.array([dFdx,dFdz,dFda,dFddx,dFddz,dFdda]).T; # with respect to x,z,theta,x',z',theta'
        G = np.array([dFdp,dFdb]).T; # with respect to throttle and brake

        return mu,F,G
