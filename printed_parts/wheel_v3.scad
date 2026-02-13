/* [Rendering] */
// Higher makes smoother circles. Use roughly 2x diameter for smooth prints.
$fn = 36; // *4 for final

overlap_cut = 0.001;

// DC N20 motor dimensions
//https://botland.store/n20-micro-motors-mp-series-medium-power/12606-micro-motor-n20-bt44-250-1-90rpm-6v-5904422306731.html
n20_shaft_length = 8;
n20_height = 10;
n20_width = 12;
n20_depth = 27;

// Wheel size
hub_outer_radius = 16; 


module n20_motor() {
    union() {
        color("silver")
        translate([-n20_height/2,-n20_width/2,-n20_depth])
        cube([n20_height, n20_width, n20_depth]);
        
        // SHAFT SETTINGS
        // ------------------------------------------
        // Measured diameter of your motor shaft (usually 3mm)
        shaft_diameter = 3.0; 

        // The "D" thickness (Distance from flat surface to opposite curved wall)
        // For a 3mm shaft, this is usually 2.5mm. Measure yours with calipers!
        shaft_flat_thickness = 2.5; 
        

        color("silver")
        translate([0,0,-overlap_cut])
        intersection() {
            translate([0,0,n20_shaft_length/2 + overlap_cut])
            cylinder(h=n20_shaft_length + 2*overlap_cut, d=shaft_diameter, center=true);
            translate([-shaft_diameter/2, -shaft_diameter/2, 0])
            cube([shaft_flat_thickness,shaft_diameter,n20_shaft_length + 2*overlap_cut]);
        }
    }
 }

// TODO: Subtract
n20_motor();

// Hard plastic
difference() {
    color("FireBrick")
    cylinder(h=n20_shaft_length, r=hub_outer_radius, $fn = 6);

    // Groove
    translate([0,0,n20_shaft_length/2])
    rotate_extrude(convexity = 10, $fn=6)
    translate([hub_outer_radius + n20_shaft_length * .17, 0, 0])
    circle(d = n20_shaft_length * (2/3), $fn=36);
    
    n20_motor();
}