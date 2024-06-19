# colreg_nav
The collision regulation repository

# Checking for an inevitable collision

THIS AN OBJECT THAT HAS BEEN DETECTED BY THE SAILBOAT VISION 

1. **Define Bounding Circles:** We use the longest distance to an edge to define the radius of a bounding circle for both the sailboat and the object.
2. **Convert to Velocity Vectors:** Translate the headings and speeds into velocity vectors.
3. **Calculate Relative Motion:** Determine the relative position and velocity of the object with respect to the boat.
4. **Check Collision Condition:** Formulate the condition where the distance between the two circles is less than or equal to the sum of their radii.
5. **Solve for Interference:** Use a quadratic equation to check if and when the relative motion results in a future collision or interference.
