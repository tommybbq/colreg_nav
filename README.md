# colreg_nav
The collision regulation repository. It builds off of the previous guide-nav repository

**guide-nav** repository contains:
- Global Path Planning (takes environmental information and a destination and solves global feasible path)
- Local Path Planning (takes set of waypoints from Global Path Planning and finds how next waypoint is reached)
- Low-Level Control (used to control Boat to next waypoint from Local Path Planning; Heading and Speed Control)


# Checking for an inevitable collision

THIS IS AN OBJECT THAT HAS BEEN DETECTED BY THE SAILBOAT VISION.

**Scenarios:**
- object is stationary and is not in the boat's route --> do nothing
- object is moving and is NOT going to intersect the boat's trajectory ---> do nothing
- object is moving and IS going to intersect the boat's trajectory at some point ---> ‼️
     - check if a collision is imminent using the steps below 

1. **Define Bounding Circles:** We use the longest distance to an edge to define the radius of a bounding circle for both the sailboat and the object.
2. **Convert to Velocity Vectors:** Translate the headings and speeds into velocity vectors.
3. **Calculate Relative Motion:** Determine the relative position and velocity of the object with respect to the boat.
4. **Check Collision Condition:** Formulate the condition where the distance between the two circles is less than or equal to the sum of their radii.
5. **Solve for Interference:** Use a quadratic equation to check if and when the relative motion results in a future collision or interference.

    - stop moving until object has passed ---> turn upwind (no-go zone) and slow down
