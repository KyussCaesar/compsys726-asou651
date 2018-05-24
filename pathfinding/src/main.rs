// common stuff for the assignment.
extern crate common;

use common::prelude::*;

use msg::
{
    geometry_msgs,
};

fn main() -> Result<(), rosrust::error::Error>
{
    rosrust::init("pathfinder");
    println!("pathfinder init");

    // init the subscriber and set up callback
    let mut _pub = rosrust::publish("/cmd_vel")?;

    let mut rate = rosrust::rate(10.0);

    println!("spinning...");

    while rosrust::is_ok()
    {
        let mut msg = geometry_msgs::Twist::default();

        msg.angular.z = 2.0;
        msg.linear.x = 0.2;

        _pub.send(msg)?;
        rate.sleep();
    }

    Ok(())
}
