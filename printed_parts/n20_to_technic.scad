include <Technic.scad>
include <BOSL2/std.scad>

$fn = 120;

overlap_cut = 0.002;
nozzle_d = 0.4;

// DC N20 motor dimensions
//https://botland.store/n20-micro-motors-mp-series-medium-power/12606-micro-motor-n20-bt44-250-1-90rpm-6v-5904422306731.html
n20_shaft_length = 8;
n20_height = 10;
n20_width = 12;
n20_depth = 27;

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
 

 
 
 
// cylinder(h=n20_shaft_length+1, r=2.95);
 
include <BOSL2/std.scad>

difference() { 
    union() {
        technic_axle( length = 3, stop = false );
        color("blue")
        translate([0,0,(n20_shaft_length+1)/2])
        cyl(l=n20_shaft_length+1, r=2.486+.5, rounding1=0, rounding2=1, $fa=1, $fs=1);
    }
    n20_motor();
}