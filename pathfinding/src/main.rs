//! # `pathfinding` node.
//!
//! This node is responsible for pathfinding within the warehouse. The node
//! subscribes to the `/map` and `/ropose` topics, and emits
//! `geometry_msgs/Twist` messages on the `/cmd_vel`.
//!
//! ## Operation
//!
//! The node sets up callbacks which listen on `/map` and `/ropose`. These
//! callbacks are implemented as closures, and are both provided with a pointer
//! to a reference-counted `Commander` value.
//! 
//! The `Commander` is built up of a `map: Option<Map>`, a `pose: Option<Pose>`,
//! and a Publisher handle. The `map` and `pose` parameters are initialised to
//! `None`. When a message on the aforementioned topics is recieved, the
//! respective field of the `Commander` is filled with `Some(_)`.
//!
//! If the `Commander` has both a `map` and a `pose`, it will find the nearest
//! unexplored cell in the map, and emit commands required to move towards it
//! on `/cmd_vel`. The commander then resets it's internal `map` and `pose` back
//! to `None`.
//!
//! The `Commander` essentially acts as a pair of queues, each of length 1. If
//! the `Commander` is missing either the latest version of the map or of the
//! robot's pose, then it will wait until it does so.
//!
//! Note: In the code, the `Commander` is also wrapped in a `Mutex`. This
//! allows for safe use of the `Commander` from both callbacks.

// common stuff for the assignment.
extern crate common;

// these are required to get safe shared mutability.
use std::sync::Arc;
use std::sync::Mutex;

use common::prelude::*;
use map_utils::
{
    Map,
    Points,
    par_transform,
    filter_map,
    extract_groups,
    neighbours,
};

use msg::geometry_msgs;
use geometry_msgs::Pose2D as Pose;

use rosrust::api::raii::Publisher as Publisher;

/// Publishes commands to the `/cmd_vel` topic, based on the robot's current
/// position in the map.
struct Commander
{
    map: Option<Map>,
    pose: Option<Pose>,
    publish: Publisher<geometry_msgs::Twist>,
}

impl Commander
{
    /// Creates a new `Commander`. `pub_handle` is a publisher that will publish
    /// to the `/cmd_vel` topic.
    pub fn new(pub_handle: Publisher<geometry_msgs::Twist>) -> Self
    {
        Commander
        {
            map: None,
            pose: None,
            publish: pub_handle,
        }
    }

    /// Publishes a velocity command to `/cmd_vel` if the commander has a recent
    /// version of the pose and map.
    pub fn update(&mut self)
    {
        println!("commander update");
        if let Commander
        {
            map: Some(map),
            pose: Some(pose),
            publish: publisher,
        } = self
        {
            println!("extracting groups");
            let groups_of_empty_cells = extract_groups(&map, |val| val <=  5, 1);
            let mut groups_of_unexp_cells = extract_groups(&map, |val| val == -1, 1);
            let mut reachable = Points::default();

            println!("finding cells");
            for (_, group_of_empty) in groups_of_empty_cells.into_iter()
            {
                for empty_cell in group_of_empty
                {
                    let mut next_empty_group = false;
                    let mut to_remove = Vec::new();

                    for (id, group_of_unexp) in &mut groups_of_unexp_cells
                    {
                        if group_of_unexp.intersection(&neighbours(empty_cell, 2)).count() > 0
                        {
                            reachable.extend(group_of_unexp.iter());
                            to_remove.push(*id);
                            next_empty_group = true;
                            break;
                        }
                    }

                    if next_empty_group
                    {
                        to_remove.into_iter().for_each(|id| {groups_of_unexp_cells.remove(&id);});
                        break;
                    }
                }
            }

            // split the map into sets of points; those that are unexplored
            // and those which are occupied.
            let unexplored = par_transform(&map, reachable);
            // let unexplored = par_transform(&map, filter_map(&map, |val| val == 01));
            let obstacles  = par_transform(&map, filter_map(&map, |val| val >   5));

            // gain
            let g = 10.0;

            // find the net attractive force
            let attractive = unexplored.par_iter()
            .map(|p| (p.0 - pose.x, p.1 - pose.y))
            .map(|p| (g * p.0 / unexplored.len() as Num, g * p.1 / unexplored.len() as Num))
            .reduce(|| (0.0, 0.0), |a,b| (a.0 + b.0, a.1 + b.1));

            // find the net repulsive force
            let repulsive = obstacles.par_iter()
            .map(|p| (p.0 - pose.x, p.1 - pose.y))
            .map(|p|
            {
                // this parameter tells you how close is too close.
                let a: Num = 0.10;

                let mag = p.0.hypot(p.1);
                let ang = p.0.atan2(p.1);

                let x1 = a.powi(2) * mag.recip() * ang.cos();
                let y1 = a.powi(2) * mag.recip() * ang.sin();

                (x1, y1)
            })
            .reduce(|| (0.0, 0.0), |a,b| (a.0 + b.0, a.1 + b.1));

            // find the net force
            let force = (attractive.0 - repulsive.0, attractive.1 - repulsive.1);

            println!("force: {:?}", force);

            // translate into robot frame.
            let robot_x = force.0 * pose.theta.cos() + force.1 * pose.theta.sin();
            let robot_y = force.1 * pose.theta.cos() - force.0 * pose.theta.sin();

            // clamp x velocity at 0. This ensures that it will never drive
            // backwards into something it hasn't seen.
            let robot_x = robot_x.max(0.0);

            // create message
            let mut msg = geometry_msgs::Twist::default();

            // set the values
            msg.linear.x  = robot_x;
            msg.angular.z = robot_y;

            // publish the command
            publisher.send(msg);
        }
    }
}

fn main() -> Result<(), rosrust::error::Error>
{
    rosrust::init("pathfinder");
    println!("pathfinder init");

    // a ref-counted pointer to a commander, protected by RwLock.
    // this is to allow safe, mutable access from both callbacks.
    let commander = Arc::new(Mutex::new(Commander::new(rosrust::publish("/cmd_vel")?)));

    // create a handle for each callback.
    // these will be moved into the closures later.
    let cmdr_handle_for_pose = commander.clone();
    let cmdr_handle_for_map  = commander.clone();

    // callback for when the pose is updated
    let pose_cb = move |pose: Pose|
    {
        println!("pose recieved");

        // aquire write permission
        // unwrap here because cbf handling poisoned locks right now.
        let mut write_lock = cmdr_handle_for_pose.lock().unwrap();

        // update pose.
        write_lock.pose = Some(pose);

        // emit new message if we have a new pose and map
        write_lock.update();
    };

    // likewise, callback for the map
    let map_cb = move |map: Map|
    {
        println!("map recieved");
        let mut write_lock = cmdr_handle_for_map.lock().unwrap();
        write_lock.map = Some(map);
    };

    // init the subscribers and set up callbacks
    let _pose_sub = rosrust::subscribe("/ropose", pose_cb)?;
    let _map_sub = rosrust::subscribe("/map", map_cb)?;

    println!("spinning...");
    rosrust::spin();

    Ok(())
}
