#include <gnc_functions.hpp>
#include <cmath> // Include the cmath library for trigonometric functions

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    // Initialize control publisher/subscribers
    init_publisher_subscriber(gnc_node);

    // Wait for FCU connection
    wait4connect();

    // Wait for user to switch to mode GUIDED
    wait4start();

    // Create local reference frame
    initialize_local_frame();

    // Request takeoff
    takeoff(3);

    // Specify circular waypoints
    std::vector<gnc_api_waypoint> waypointList;
    const int numWaypoints = 30; // Number of waypoints forming the circle
    const double radius = 10.0;   // Radius of the circle
    for (int i = 0; i < numWaypoints; ++i) {
        gnc_api_waypoint nextWayPoint;
        double angle = 2 * M_PI * i / numWaypoints; // Calculate angle for current waypoint
        nextWayPoint.x = radius * cos(angle); // Calculate x-coordinate of waypoint
        nextWayPoint.y = radius * sin(angle); // Calculate y-coordinate of waypoint
        nextWayPoint.z = 3; // Maintain constant altitude
        nextWayPoint.psi = atan2(nextWayPoint.y, nextWayPoint.x) * 180 / M_PI; // Calculate heading (in degrees) towards the next waypoint
        waypointList.push_back(nextWayPoint);
    }

    // Specify control loop rate. We recommend a low frequency to avoid overloading the FCU with messages.
    ros::Rate rate(2.0);
    int counter = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if(check_waypoint_reached(.3) == 1)
        {
            if (counter < waypointList.size())
            {
                set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                counter++;
            }
            else
            {
                // Land after all waypoints forming the circle are reached
                land();
            }
        }
    }

    return 0;
}