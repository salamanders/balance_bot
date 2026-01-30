// ==============================================
// AIRLESS TIRE - EXPOSED VANE TREAD
// ==============================================

// NOTE: Remember to print as mirror, not just 2 copies!

/* [Dimensions] */
// The total width of the tire across the tread
tire_width = 8;

// Radius of the area around the D shaft (with extra)
axle_radius = 4; 

// Outer radius of the rigid hub zone
hub_outer_radius = 7; 

// Outer radius of the compliant soft zone
soft_outer_radius = 24;

// Final outer radius of the tire (the tread surface)
tire_outer_radius = 30;  


/* [Tread Configuration] */
tread_extension = 1.0; // How far the vanes poke out (1.0 to 1.5mm is the sweet spot)

nozzle_diameter = 0.4;
min_wall_thickness = nozzle_diameter * 2;


/* [Vane Configuration] */
num_vanes = 36; // Increased slightly to ensure smooth rolling on the tips
vane_thickness = 2*nozzle_diameter; 

// Controls the "V" shape on the tread (Ground smoothness)
chevron_angle = 35; 

// Controls the "Lean" of the soft (Torque Stiffness)
// 15-20 degrees is usually enough. Too high and it gets too stiff vertically.
truss_angle = 55; 


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
    
    

/* [Rendering] */
// Higher makes smoother circles. Use roughly 2x diameter for smooth prints.
$fn = tire_outer_radius*1; // *4 for final)

// --- Helper Modules ---
module ring(height, r_outer, r_inner) {
    difference() {
        cylinder(h=height, r=r_outer, center=true);
        cylinder(h=height+1, r=r_inner, center=true);
    }
}


/*
// ==============================================
// Generates the geometry for the interlocking teeth.
// We call this twice: once to ADD to the hub, once to SUBTRACT from the cushion.
module interlock_teeth() {
    num_teeth = 12;      // 12 teeth is plenty for grip
    tooth_depth = 4.0;   // How deep they bite into the cushion (mm)
    tooth_width = 4.0;   // Thickness of the tooth
    
    intersection() {
        union() {
            cylinder(h=tire_width/2, r1=hub_outer_radius+2, r2=hub_outer_radius+10);
    rotate([180,0,0])
            cylinder(h=tire_width/2, r1=hub_outer_radius+2, r2=hub_outer_radius+10);
        }
        union() {
        for (i = [0 : num_teeth-1]) {
            rotate([0, 0, i * (360 / num_teeth)]) 
            translate([hub_outer_radius, 0, 0]) 
            // We center the cube on the boundary line so half is in, half is out.
            // But for the Hub, we only care about the part sticking OUT.
            translate([tooth_depth/2, 0, 0]) 
            cube([tooth_depth+1, tooth_width, tire_width], center=true);
        }
        }
    }
}
*/


// --- ZONE 1: THE HUB (Red) ---
// Intended Slicer Settings: 100% Solid Infill, high wall count.
// Function: Rigid connection to motor.
module Zone1_Hub() {
    color("FireBrick")
    union() {
        translate([0,0,0.02/2])
        ring(height=tire_width+0.02, r_outer=hub_outer_radius, r_inner=axle_radius);
        
        translate([0,0,0.02/2])
        ring(height=tire_width, r_outer=hub_outer_radius, r_inner=axle_radius);
        for (i = [0:60:360]) {
            difference() {
                rotate([0,0,i])
                cube([(hub_outer_radius+5)*2, 4, tire_width], center=true);
                cylinder(h=tire_width+0.02, r=axle_radius, center=true);
            }
        }
        
        difference() {
            cylinder(h=tire_width+0.02, r=hub_outer_radius, center=true);
        
            // 2. The D-Shaft Cutout (SUBTRACT)
            translate([0,0, -50]) // Center the cut vertically
            linear_extrude(height=100) {
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
            
        //interlock_teeth();
    }
}


// --- ZONE 2: COMPLIANT softS (Blue) ---
// Intended Slicer Settings: ~10% Gyroid Infill. 0 Top/Bottom Layers.
// Function: Shock absorption for rocks.
// Note: We start exactly where the hub ends.
// "Infill/Perimeter Overlap" to 30%
module Zone2_softs() {
    color("SteelBlue")
    difference() {
        translate([0,0,0.01/2])
        ring(height=tire_width+0.01, r_outer=soft_outer_radius, r_inner=hub_outer_radius);
        scale([1,1,2])
        Zone1_Hub();
    }
}


// --- ZONE 3: SHEAR BAND & TREAD (Green) ---
// You might want 2-3 top/bottom solid layers here for actual grip surface.
// Function: Distribute load evenly, prevent torsional wind-up, provide grip.
// Note: This is the "centimeter worth of vanes" and final layer combined.
// Set Perimeters to 4 or 5.
// ==============================================
// ZONE 3: THE A-FRAME (TORQUE + SQUISH + NO MESH)
// ==============================================

half_way_between_soft_and_tire_r = soft_outer_radius + (tire_outer_radius-soft_outer_radius)/2;
gap_between_soft_and_tire = tire_outer_radius - soft_outer_radius - nozzle_diameter*6;
    
module vanes() {
    chamfer_depth = 1.0;
    difference() {        
        intersection() {
            union() {
            for (i = [0 : num_vanes-1]) {
                rotate([0, 0, i * (360 / num_vanes)]) 
                translate([(soft_outer_radius + tire_outer_radius) / 2, 0, 0]) 
                // Compound Angle: Tilt for Tread + Lean for Truss
                rotate([chevron_angle, 0, -truss_angle]) 
                color("green")
                cube([tire_outer_radius*1.6, vane_thickness, tire_width*3], center=true);
            }
            
            
            // off-angle treads
            intersection() {
                color("purple")
                    for (i = [0 : num_vanes-1]) {
                    rotate([6.5, 0, i * (360 / num_vanes)]) 
                    translate([(soft_outer_radius + tire_outer_radius) / 2, 0, 0]) 
                    // Compound Angle: Tilt for Tread + Lean for Truss
                    rotate([chevron_angle*0.95, 0, 0]) 
                    cube([tire_outer_radius*1.6, vane_thickness, tire_width*3], center=true);
                }
                color("blue")
                difference() {
                    cylinder(h=tire_width/2, r=tire_outer_radius+tread_extension);
                    cylinder(h=tire_width/2+1, r=tire_outer_radius);
                }
                
            }
            
            
            }
          
            // old cut, sharp edge: cylinder(h=tire_width/2, r=tire_outer_radius+tread_extension);
            
            // 2. THE CHAMFERED CUTTER (Replaces simple cylinder)
            // This creates the tire profile: Straight wall -> 45deg Taper
            union() {
                // The main block (Straight Wall)
                color("green")
                cylinder(h=(tire_width/2) - chamfer_depth, r=tire_outer_radius+tread_extension);
                
                // The Taper (Chamfer)
                translate([0, 0, (tire_width/2) - chamfer_depth])
                color("green")
                cylinder(h=chamfer_depth, 
                         r1=tire_outer_radius+tread_extension, 
                         r2=tire_outer_radius+tread_extension - chamfer_depth);
            }
            
        }
        cylinder(h=tire_width+0.1, r=soft_outer_radius, center=true);
        // Keep the vanes from touching along the spine
        rotate_extrude(convexity = 10)
            translate([half_way_between_soft_and_tire_r, 0, 0])
                circle(d = gap_between_soft_and_tire, $fn=36);
    }
}


module Zone3_ShearVanes() {
    intersection() {
        // Need to split for fuzzy tires
        cylinder(h=tire_width, r=tire_outer_radius-min_wall_thickness+0.4, center=true);
        union() {
            // Drums (outer)
            ring(height=tire_width, r_outer=tire_outer_radius, r_inner=tire_outer_radius - min_wall_thickness);
            // 1. The Drums (Inner)
            ring(height=tire_width, r_outer=soft_outer_radius + min_wall_thickness, r_inner=soft_outer_radius);
            vanes();
            mirror([0,0,1])
            vanes();
        }
    }
}

module Zone4_Outer() {
    difference() {
        union() {
            // Drums (outer)
            ring(height=tire_width, r_outer=tire_outer_radius, r_inner=tire_outer_radius - min_wall_thickness);
            // 1. The Drums (Inner)
            ring(height=tire_width, r_outer=soft_outer_radius + min_wall_thickness, r_inner=soft_outer_radius);
            vanes();
            mirror([0,0,1])
            vanes();
        }
        // Need to split for fuzzy tires
        cylinder(h=tire_width, r=tire_outer_radius-min_wall_thickness+0.4, center=true);
    }
}

// ==============================================
// DISPLAY
// ==============================================


color("FireBrick")
union(){ 
    Zone1_Hub();
}

color("SteelBlue")
union() {
    Zone2_softs();
}

color("green")
union() {
    Zone3_ShearVanes();
}

color("brown")
union() {
    Zone4_Outer();
}



