#include "golfcart_pilot/PilotNode.h"

void PilotNode::absoluteCommandCallback(const geometry_msgs::Twist::ConstPtr &m) {
    if (!ax1500) {
        ROS_WARN("ax1500 connection lost");
        connect();
    }

    uint8_t value(sParams.encCen + sParams.ticPerRad * m->linear.y);
    if (value < sParams.encMin) {
        ROS_WARN("Trying to turn to %d, clamping at %d", value, sParams.encMin);
        value = sParams.encMin;
    } else if (value > sParams.encMax) {
        ROS_WARN("Trying to turn to %d, clamping at %d", value, sParams.encMax);
        value = sParams.encMax;
    }
    roboteq_ax1500::channel_forward cmd;
    cmd.request.channel = sParams.chan;
    cmd.request.value = value;
    ROS_INFO("Sending channel_forward to ax1500: channel %d, value %d",
            cmd.request.channel, cmd.request.value);
    if (ax1500.call(cmd)) {
        state = *m;
    } else {
        ROS_ERROR("AX1500 call failed");
        ROS_WARN("Not changing state");
    }
}

void PilotNode::relativeCommandCallback(const geometry_msgs::Twist::ConstPtr &m) {
}

bool PilotNode::connect() {
    ros::Rate r(1);
    do {
        ROS_INFO("Connecting to ax1500");
        ax1500 = n.serviceClient<roboteq_ax1500::channel_forward>("channel_forward", true);
        r.sleep();
    } while (!ax1500 && ros::ok());

    if (ax1500) {
        ROS_INFO("Connected to ax1500");
        return true;
    } else {
        ROS_ERROR("Couldn't connect to ax1500");
        return false;
    }
}

bool PilotNode::init() {
    if (!connect())
        return false;

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

    n.getParam("/golfcart_pilot/steering_channel", sParams.chan);
    n.getParam("/golfcart_pilot/steering_max_enc", sParams.encMax);
    n.getParam("/golfcart_pilot/steering_center_enc", sParams.encCen);
    n.getParam("/golfcart_pilot/steering_min_enc", sParams.encMin);
    n.getParam("/golfcart_pilot/steering_range", sParams.range);
    ROS_INFO("Using: max %d tics, center %d tics, min %d tics, range %f radians",
            sParams.encMax, sParams.encCen, sParams.encMin, sParams.range);

    sParams.radPerTic = sParams.range / (sParams.encMax - sParams.encMin);
    sParams.ticPerRad = 1 / sParams.radPerTic;
    sParams.minRad = (sParams.encMin - sParams.encCen) * sParams.radPerTic;
    sParams.maxRad = (sParams.encMax - sParams.encCen) * sParams.radPerTic;

    n.setParam("/golfcart_pilot/rad_per_tic", sParams.radPerTic);
    n.setParam("/golfcart_pilot/tic_per_rad", sParams.ticPerRad);
    n.setParam("/golfcart_pilot/steering_max_rad", sParams.maxRad);
    n.setParam("/golfcart_pilot/steering_min_rad", sParams.minRad);

    ROS_INFO("Radians Per Encoder Tic: %f", sParams.radPerTic);
    ROS_INFO("Max steering: %f radians, Min steering: %f radians", sParams.maxRad, sParams.minRad);

    absCmdSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/abs_cmd", 100,
            &PilotNode::absoluteCommandCallback, this);
    relCmdSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/rel_cmd", 100,
            &PilotNode::relativeCommandCallback, this);

    return true;
}
