// ==============================================
// AIRLESS TIRE - EXPOSED VANE TREAD
// ==============================================

// NOTE: Remember to print as mirror, not just 2 copies!

/* [Dimensions] */
// The total width of the tire across the tread
tire_width = 25;

// Radius of the motor shaft hole
axle_radius = 4; 

// Outer radius of the rigid hub zone
hub_outer_radius = 16; 

// Outer radius of the compliant spoke zone
spoke_outer_radius = 38;

// Final outer radius of the tire (the tread surface)
tire_outer_radius = 48;  


/* [Tread Configuration] */
tread_extension = 1.0; // How far the vanes poke out (1.0 to 1.5mm is the sweet spot)

nozzle_diameter = 0.4;
min_wall_thickness = nozzle_diameter * 3;


/* [Vane Configuration] */
num_vanes = 36; // Increased slightly to ensure smooth rolling on the tips
vane_thickness = 2*nozzle_diameter; 

// Controls the "V" shape on the tread (Ground smoothness)
chevron_angle = 35; 

// Controls the "Lean" of the spoke (Torque Stiffness)
// 15-20 degrees is usually enough. Too high and it gets too stiff vertically.
truss_angle = 55; 


/* [Rendering] */
// Higher makes smoother circles. Use roughly 2x diameter for smooth prints.
$fn = tire_outer_radius*4; // *4 for final)

// --- Helper Modules ---
module ring(height, r_outer, r_inner) {
    difference() {
        cylinder(h=height, r=r_outer, center=true);
        cylinder(h=height+1, r=r_inner, center=true);
    }
}

// --- ZONE 1: THE HUB (Red) ---
// Intended Slicer Settings: 100% Solid Infill, high wall count.
// Function: Rigid connection to motor.
module Zone1_Hub() {
    color("FireBrick") {
    translate([0,0,0.02/2])
        ring(height=tire_width+0.02, r_outer=hub_outer_radius, r_inner=axle_radius);
    }
}

// --- ZONE 2: COMPLIANT SPOKES (Blue) ---
// Intended Slicer Settings: ~10% Gyroid Infill. 0 Top/Bottom Layers.
// Function: Shock absorption for rocks.
// Note: We start exactly where the hub ends.
// "Infill/Perimeter Overlap" to 30%
module Zone2_Spokes() {
    color("SteelBlue") {
    translate([0,0,0.01/2])
        ring(height=tire_width+0.01, r_outer=spoke_outer_radius, r_inner=hub_outer_radius);
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

half_way_between_spoke_and_tire_r = spoke_outer_radius + (tire_outer_radius-spoke_outer_radius)/2;
gap_between_spoke_and_tire = tire_outer_radius - spoke_outer_radius - nozzle_diameter*6;
    
module vanes() {
    chamfer_depth = 1.0;
    difference() {        
        intersection() {
            union() {
            for (i = [0 : num_vanes-1]) {
                rotate([0, 0, i * (360 / num_vanes)]) 
                translate([(spoke_outer_radius + tire_outer_radius) / 2, 0, 0]) 
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
                    translate([(spoke_outer_radius + tire_outer_radius) / 2, 0, 0]) 
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
        cylinder(h=tire_width+0.1, r=spoke_outer_radius, center=true);
        // Keep the vanes from touching along the spine
        rotate_extrude(convexity = 10)
            translate([half_way_between_spoke_and_tire_r, 0, 0])
                circle(d = gap_between_spoke_and_tire, $fn=36);
    }
}


module Zone3_ShearVanes() {
    // 1. The Drums (Inner & Outer)
    ring(height=tire_width, r_outer=spoke_outer_radius + min_wall_thickness, r_inner=spoke_outer_radius);
    ring(height=tire_width, r_outer=tire_outer_radius, r_inner=tire_outer_radius - min_wall_thickness);

    vanes();
    mirror([0,0,1])
    vanes();
}


// ==============================================
// DISPLAY
// ==============================================

union(){ 
    Zone1_Hub();
}
union() {
    Zone2_Spokes();
}
color("green")
union() {
    Zone3_ShearVanes();
}






