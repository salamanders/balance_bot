/* [Rendering] */
// Higher makes smoother circles. Use roughly 2x diameter for smooth prints.
$fn = 36; // *4 for final)

// Build outwards from the motor
overlap = 0.001;

// DC N20 motor -bmw dimensions
silver_height = 10;
silver_width = 12;
silver_depth = 27;


// SHAFT SETTINGS
// ------------------------------------------
// Measured diameter of your motor shaft (usually 3mm)
shaft_diameter = 3.0; 

// The "D" thickness (Distance from flat surface to opposite curved wall)
// For a 3mm shaft, this is usually 2.5mm. Measure yours with calipers!
shaft_flat_thickness = 2.5; 

// Printing Tolerance (Crucial!)
// 0.15mm - 0.2mm is usually good for a snug press fit.
fit_tolerance = 0.2;

shaft_length = 8;
//https://botland.store/n20-micro-motors-mp-series-medium-power/12606-micro-motor-n20-bt44-250-1-90rpm-6v-5904422306731.html

// Weel sections, all from center
press_fit_r = 6;
squish_r = 20;
paddle = squish_r + 10;

nozzle_diameter = 0.4;
wall_thickness = nozzle_diameter * 3;
num_vanes = 36; // Increased slightly to ensure smooth rolling on the tips
vane_thickness = 2*nozzle_diameter; 

// Controls the "V" shape on the tread (Ground smoothness)
chevron_angle = 35; 

// Controls the "Lean" of the soft (Torque Stiffness)
// 15-20 degrees is usually enough. Too high and it gets too stiff vertically.
truss_angle = 55; 


module zmirror(){
    children();
    mirror([0,0,1])
	children();
}

// 2. The D-Shaft Cutout (SUBTRACT)
module DShaft() {
    color("silver")
    translate([0,0, -shaft_length/2 - overlap]) // Center the cut vertically
    linear_extrude(height=shaft_length + overlap*2) {
        intersection() {
            // A. The Round Shaft
            circle(d = shaft_diameter + fit_tolerance, $fn=60);
            
            // B. The Flat Cut (The D-shape)
            // We draw a large square shifted to create the flat edge
            translate([ -shaft_diameter, -shaft_diameter ]) 
                square([ (shaft_flat_thickness + fit_tolerance) + shaft_diameter/2, shaft_diameter*2 ]);
        }
    }
}

// Motor block
*color("silver")
translate([0,0,-24/2 - shaft_length/2])
cube([10,12,24], center=true);

// Hard press-fit center
module PressFit_00() {
    color("FireBrick")
    difference() {
        intersection() {
            cylinder(h=shaft_length, r=press_fit_r, center=true, $fn=6);
            zmirror() {
                cylinder(h=shaft_length/2, r1=press_fit_r-1, r2=press_fit_r + 1, $fn=6);
            }
        }
        DShaft();
    }
}

PressFit_00();

difference() {
cylinder(h=shaft_length, r=squish_r, center=true);
cylinder(h=shaft_length+overlap*2, r=press_fit_r-1, center=true);
PressFit_00();
}