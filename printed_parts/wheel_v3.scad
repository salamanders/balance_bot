/* [Rendering] */
// Higher makes smoother circles. Use roughly 2x diameter for smooth prints.
$fn = 120; // *4 for final

overlap_cut = 0.002;
nozzle_d = 0.4;


// DC N20 motor dimensions
//https://botland.store/n20-micro-motors-mp-series-medium-power/12606-micro-motor-n20-bt44-250-1-90rpm-6v-5904422306731.html
n20_shaft_length = 8;
n20_height = 10;
n20_width = 12;
n20_depth = 27;

// Wheel size
hub_outer_radius = 16; 
spoke_outer_radius = 32; 
tire_outer_radius = 42;

module n20_motor() {
    translate([0,0,-n20_shaft_length/2])
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

// Hard plastic
module Center_HardPlastic() {
    difference() {
        // Hex to avoid spinning.
        color("FireBrick")
        cylinder(h=n20_shaft_length, r=hub_outer_radius, $fn = 6, center=true);

        // Groove.
        rotate_extrude(convexity = 10, $fn=6)
        translate([hub_outer_radius + n20_shaft_length * .17, 0, 0])
        circle(d = n20_shaft_length * (2/3));        
    }
}

// Hard parts, PLA or PET-G
difference() {
    Center_HardPlastic();
    #n20_motor();
}


module HollowCylinder(h, r, r_inner) {
    difference() {
        cylinder(h=h, r=r, center=true);
        cylinder(h=h+overlap_cut, r=r_inner, center=true);
    }
}


num_vanes =36;
vane_thickness = nozzle_d*2; 
vane_angle = 35; 
herringbone_angle = 30;
vane_gap_r = tire_outer_radius - spoke_outer_radius;
// Extra because of the angle
vane_length = vane_gap_r * 3;
tread_r = nozzle_d*2;


module TiltedVanes(tilt=1) {
    color("purple")
    difference() {
        intersection() {
            union()
            for (i = [0 : num_vanes-1]) {
                rotate([0, 0, i * (360 / num_vanes)+0.4*tilt])
                translate([spoke_outer_radius, 0, 0])
                rotate([-herringbone_angle*tilt,0,vane_angle*tilt])
                // No Z offset, use the cylinder-carve to make a 50% herringbone
                translate([vane_length*0.2,0,0])
                cube([vane_length, vane_thickness, n20_shaft_length*2], center=true);
            }
            HollowCylinder(h=n20_shaft_length, r=tire_outer_radius+tread_r, r_inner=spoke_outer_radius);
        }
        // Notch for vanes
        translate([0,0,-n20_shaft_length*.6*tilt])
        rotate_extrude(convexity = 10)
        translate([spoke_outer_radius + vane_gap_r*.62, 0, 0])
        rotate([0,0,30])
        circle(r = vane_gap_r*.745, $fn=6); 
    }
}


module Squishy() {
    color("lightblue")
    cylinder(h=n20_shaft_length, r=spoke_outer_radius, center=true);
    
    color("ForestGreen")
    HollowCylinder(h=n20_shaft_length, r=spoke_outer_radius+nozzle_d, r_inner=spoke_outer_radius);
    
    TiltedVanes();
    
    TiltedVanes(-1);
    
    
    color("ForestGreen")
    HollowCylinder(h=n20_shaft_length, r=tire_outer_radius, r_inner=tire_outer_radius-nozzle_d*2);
    
}

 



difference() {
    Squishy();
    Center_HardPlastic();
}

/*


vane_length = vane_gap_r * sqrt(2);

module OuterVanesTop(herringbone_tilt = 1) {
    color("ForestGreen")
    cylinder(h=n20_shaft_length, r=spoke_outer_radius + nozzle_d*2);
    
    difference() {

    
    // "scoop"
    // Groove
    rotate_extrude(convexity = 10)
    translate([(tire_outer_radius + spoke_outer_radius)/2, 0, 0])
    circle(d = tire_outer_radius - spoke_outer_radius);         
    }
}


color("brown")
intersection() {
union() 
for (i = [0 : num_vanes-1]) {
    rotate([0, 0, i * (360 / num_vanes)])
    translate([spoke_outer_radius, 0, 0])
    rotate([herringbone_angle * 1,0,vane_angle])
    cube([vane_length +4, vane_thickness, n20_shaft_length], center=true);
}
cylinder(h=n20_shaft_length, r=tire_outer_radius);
}


module OuterVanes(){
    OuterVanesTop();
    translate([0,0,n20_shaft_length])
    rotate([180,0,6.8])
	OuterVanesTop(-1);
}



// Squishy parts, TPU
union() {
    difference() {
        OuterVanes();
        Middle_Squishy();
    }
    difference() {
        Middle_Squishy();
        Center_HardPlastic();
    }
}
*/