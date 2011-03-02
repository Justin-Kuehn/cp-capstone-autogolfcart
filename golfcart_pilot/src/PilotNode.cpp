#include "golfcart_pilot/PilotNode.h"

void PilotNode::absoluteCommandCallback(const geometry_msgs::Twist::ConstPtr &m) {
    geometry_msgs::Twist nM(*m);
    uint8_t steer(stParams.encCen + stParams.ticPerRad * m->angular.y);
    if (steer < stParams.encMin) {
        ROS_WARN("Trying to turn to %d, clamping at %d", steer, stParams.encMin);
        steer = stParams.encMin;
        nM.angular.y = stParams.minRad;
    } else if (steer > stParams.encMax) {
        ROS_WARN("Trying to turn to %d, clamping at %d", steer, stParams.encMax);
        steer = stParams.encMax;
        nM.angular.y = stParams.maxRad;
    }

    uint16_t speed(spParams.min + spParams.ticPerPercent * m->linear.x);
    if (speed > spParams.max) {
        ROS_WARN("Trying to accelerate to %d, clamping at %d", speed, spParams.max);
        speed = spParams.max;
        nM.linear.x = 100;
    } else if (speed <= spParams.min) {
        speed = 0;
        nM.linear.x = 0;
    }

    if (sendSteeringCommand(steer) && sendSpeedCommand(speed)) {
        state = nM;
    } else {
        ROS_ERROR("Command failed!");
        ROS_WARN("Trying to stop");
        while (!sendSteeringCommand(stParams.encCen) || !sendSpeedCommand(0)) {}
    }
}

bool PilotNode::sendSteeringCommand(uint8_t value) {
    if (!ax1500) {
        ROS_WARN("ax1500 connection lost");
        connect();
    }

    roboteq_ax1500::channel_forward cmd;
    cmd.request.channel = stParams.chan;
    cmd.request.value = value;
    ROS_INFO("Sending channel_forward to ax1500: channel %d, value %d",
            cmd.request.channel, cmd.request.value);
    return ax1500.call(cmd);
}

bool PilotNode::sendSpeedCommand(uint8_t value) {
    if (!arduino) {
        ROS_WARN("arduino connection lost");
        connect();
    }

    pmad::Switch cmd;
    cmd.request.channel = 5;
    cmd.request.state = value;
    ROS_INFO("Sending pmad_switch_control to arduino: channel %d, value %d",
            cmd.request.channel, cmd.request.state);

    return arduino.call(cmd);
}

void PilotNode::relativeCommandCallback(const geometry_msgs::Twist::ConstPtr &m) {
    geometry_msgs::Twist::Ptr nState(new geometry_msgs::Twist());
    nState->angular.x = state.angular.x + m->angular.x;
    nState->angular.y = state.angular.y + m->angular.y;
    nState->angular.z = state.angular.z + m->angular.z;
    nState->linear.x = state.linear.x + m->linear.x;
    nState->linear.y = state.linear.y + m->linear.y;
    nState->linear.z = state.linear.z + m->linear.z;
    absoluteCommandCallback(nState);
}

bool PilotNode::connect() {
    bool ret(true);

    ros::Rate r(1);
    do {
        ROS_INFO("Connecting to ax1500");
        ax1500 = n.serviceClient<roboteq_ax1500::channel_forward>("channel_forward", true);
        r.sleep();
    } while (!ax1500 && ros::ok());

    if (ax1500) {
        ROS_INFO("Connected to ax1500");
    } else {
        ROS_ERROR("Couldn't connect to ax1500");
        ret &= false;
    }

    do {
        ROS_INFO("Connecting to arduino");
        arduino = n.serviceClient<pmad::Switch>("pmad_switch_control", true);
        r.sleep();
    } while (!arduino && ros::ok());

    if (arduino) {
        ROS_INFO("Connected to arduino");
    } else {
        ROS_ERROR("Couldn't connect to arduino");
        ret &= false;
    }

    return ret;
}

bool PilotNode::init() {
    if (!n.hasParam("/golfcart_pilot/steering_channel")) {
        ROS_ERROR("No steering channel");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_min_enc")) {
        ROS_ERROR("No min steering encoder value");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_center_enc")) {
        ROS_ERROR("No center steering encoder");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_max_enc")) {
        ROS_ERROR("No max steering encoder");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_range")) {
        ROS_ERROR("No steering range");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/speed_min")) {
        ROS_ERROR("No min speed");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/speed_max")) {
        ROS_ERROR("No max speed");
        return false;
    }



    n.getParam("/golfcart_pilot/steering_channel", stParams.chan);
    n.getParam("/golfcart_pilot/steering_max_enc", stParams.encMax);
    n.getParam("/golfcart_pilot/steering_center_enc", stParams.encCen);
    n.getParam("/golfcart_pilot/steering_min_enc", stParams.encMin);
    n.getParam("/golfcart_pilot/steering_range", stParams.range);
    ROS_INFO("Using: max %d tics, center %d tics, min %d tics, range %f radians",
            stParams.encMax, stParams.encCen, stParams.encMin, stParams.range);

    stParams.radPerTic = stParams.range / (stParams.encMax - stParams.encMin);
    stParams.ticPerRad = 1 / stParams.radPerTic;
    stParams.minRad = (stParams.encMin - stParams.encCen) * stParams.radPerTic;
    stParams.maxRad = (stParams.encMax - stParams.encCen) * stParams.radPerTic;

    n.setParam("/golfcart_pilot/rad_per_tic", stParams.radPerTic);
    n.setParam("/golfcart_pilot/tic_per_rad", stParams.ticPerRad);
    n.setParam("/golfcart_pilot/steering_max_rad", stParams.maxRad);
    n.setParam("/golfcart_pilot/steering_min_rad", stParams.minRad);

    ROS_INFO("Radians Per Encoder Tic: %f", stParams.radPerTic);
    ROS_INFO("Max steering: %f radians, Min steering: %f radians", stParams.maxRad, stParams.minRad);

    n.getParam("/golfcart_pilot/speed_min", spParams.min);
    n.getParam("/golfcart_pilot/speed_max", spParams.max);
    ROS_INFO("Using: min %d tics, max %d tics", spParams.min, spParams.max);

    spParams.ticPerPercent = (spParams.max - spParams.min) / 100;
    ROS_INFO("Tics Per Percent: %f", spParams.ticPerPercent);

    if (!connect())
        return false;

    if (sendSteeringCommand(stParams.encCen)) {
        state.angular.x = 0;
        state.angular.y = 0;
        state.angular.z = 0;
        state.linear.x = 0;
        state.linear.y = 0;
        state.linear.z = 0;
    } else {
        ROS_ERROR("Couldn't reset state");
        return false;
    }

    absCmdSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/abs_cmd", 100,
            &PilotNode::absoluteCommandCallback, this);
    relCmdSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/rel_cmd", 100,
            &PilotNode::relativeCommandCallback, this);

    return true;
}
