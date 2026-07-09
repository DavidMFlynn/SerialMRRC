// **********************************
// Servo Switch Machine
//
// License: MIT License
// Filename: ServoSM.scad
// Created: 7/6/2026
// Revision: 1.0.1   7/8/2026
// 
// **********************************
//  ***** Notes *****
//
// Uses an SG90 or equivalent micro servo as a switch machine.
// 0.047" (1.2mm) Misic wire is recommended.
// Mounted to the layout with 2 #6x1/2" Pan Head Sheet Metal Screws.
//
//  ***** History *****
//
// 1.0.1   7/8/2026  Code cleanup
// 1.0.0   7/6/2026  First code
//
// **********************************
//  ***** for STL output *****
//
// SwitchMachineBase();
// ServoHornExtension();
//
//  *** Tools ***
//
// MountingHoleJig();
// WireBendingJig();
// TestFixtureAlpha(Baseboard_t=15);
//
// **********************************
include<CommonStuffSAEmm.scad>

Overlap=0.05;
IDXtra=0.2;
$fn=90;

Servo_X=12.2;
Servo_Y=22.6;
Servo_Z=27;
ServoDeckTop_Z=18.4;
ServoDeck_t=2.6;
ServoDeck_Y=32.4;
Servo_r=0.5;
ServoMH_cl=28;
ServoHornTop_Z=32.5;
ServoHornOffset_Y=5.8;

module Servo(XtraTop=0){
	BodyHole_X=Servo_X+IDXtra*2;
	BodyHole_Y=Servo_Y+IDXtra*2;
	
	RoundRect(X=BodyHole_X, Y=BodyHole_Y, Z=Servo_Z, R=Servo_r);
	
	translate([0,0,ServoDeckTop_Z-ServoDeck_t]) RoundRect(X=BodyHole_X, Y=ServoDeck_Y+IDXtra*2, Z=ServoDeck_t, R=Servo_r);

	translate([0,ServoHornOffset_Y,ServoHornTop_Z]) rotate([180,0,0]) RoundRect(X=Servo_X, Y=32, Z=6, R=Servo_r);
	
	translate([0, ServoMH_cl/2, Servo_Z+10]) Bolt2Hole(depth=Servo_Z+10);
	translate([0, -ServoMH_cl/2, Servo_Z+10]) Bolt2Hole(depth=Servo_Z+10);
} // Servo

// Servo(XtraTop=10);

module SwitchMachineBase(){
	Width=22;
	Len=46;
	MountingPlate_Y=Len+20;
	MountingPlate_t=2.1; // 7 layers
	ControlRod_Y=Len/2-5;
	Plate_H=18;
	ServoBottom_Z=MountingPlate_t+ServoHornTop_Z+3;
	ServoOffset_Y=-2.5;
	MountingHole_bs=Len+10;
	ControlRodEnd_Y=-Len/2+2.5;
	Wire_d=1.2;
	
	difference(){
		union(){
			RoundRect(X=Width, Y=MountingPlate_Y, Z=MountingPlate_t, R=5);
			hull(){
				RoundRect(X=Width, Y=Len, Z=MountingPlate_t, R=2);
				RoundRect(X=Servo_X+5, Y=Len, Z=Plate_H+MountingPlate_t, R=2);
			} // hull
		} // union
		
		// Wire hole
		translate([0,ControlRodEnd_Y,-Overlap]) cylinder(d=Wire_d+IDXtra*2, h=50);
		
		// Wire clearance
		hull(){
			translate([0,ControlRodEnd_Y,-Overlap]) cylinder(d=3, h=3);
			translate([0,ControlRod_Y,-Overlap]) cylinder(d=7.5, h=3);
		} // hull
		
		// Servo horn clearance
		//translate([0,4,-Overlap]) cylinder(d=20, h=12+Overlap);
		//translate([0,4,12-Overlap]) cylinder(d1=20, d2=15, h=4);
		hull(){
			translate([0,0,-Overlap]) cylinder(d=16, h=15);
			translate([0,ControlRod_Y,-Overlap]) scale([1,0.5,1]) cylinder(d=16, h=15);
		} // hull
		
		translate([0,ServoOffset_Y,ServoBottom_Z]) rotate([0,180,0]) Servo(XtraTop=10);
		
		translate([0,ControlRod_Y,-Overlap]) scale([1,0.8,1]) cylinder(d=9, h=50);
		
		// Mounting bolts
		translate([0,MountingHole_bs/2,MountingPlate_t]) Bolt6ClearHole();
		translate([0,-MountingHole_bs/2,MountingPlate_t]) Bolt6ClearHole();
	} // difference

} // SwitchMachineBase

// SwitchMachineBase();

module MountingHoleJig(){
	Width=22;
	Len=46;
	MountingPlate_Y=Len+20;
	MountingPlate_t=2.1; // 7 layers
	ControlRod_Y=Len/2-5;
	Plate_H=18;
	ServoBottom_Z=MountingPlate_t+ServoHornTop_Z+3;
	ServoOffset_Y=-2.5;
	MountingHole_bs=Len+10;
	ControlRodEnd_Y=-Len/2+2.5;
	Wire_d=1.2;

	difference(){
		union(){
			RoundRect(X=Width, Y=MountingPlate_Y, Z=MountingPlate_t, R=5);
			translate([0,ControlRod_Y,MountingPlate_t-Overlap]) cylinder(d1=7, d2=6, h=2);
		} // union
		
		// Mounting bolts
		translate([0,MountingHole_bs/2,MountingPlate_t]) Bolt6Hole();
		translate([0,-MountingHole_bs/2,MountingPlate_t]) Bolt6Hole();
	} // difference
	
} // MountingHoleJig

// MountingHoleJig();

module WireBendingJig(){
	Width=22;
	Len=46;
	MountingPlate_Y=Len+20;
	MountingPlate_t=2.1; // 7 layers
	ControlRod_Y=Len/2-5;
	Plate_H=12;
	ServoBottom_Z=MountingPlate_t+ServoHornTop_Z+3;
	ServoOffset_Y=-2.5;
	MountingHole_bs=Len+10;
	ControlRodEnd_Y=-Len/2+2.5;
	Wire_d=1.2;
	BendPoint=-4.75; // from switch machine body end

	difference(){
		translate([0,BendPoint,0]) RoundRect(X=Width, Y=Len, Z=Plate_H, R=2);
		
		// Wire hole
		translate([0,ControlRodEnd_Y,-Overlap]) cylinder(d=Wire_d+IDXtra*2, h=50);
		hull(){
			translate([0,ControlRodEnd_Y,Plate_H]) rotate([-90,0,0]) cylinder(d=Wire_d+IDXtra*2, h=Len);
			translate([0,ControlRodEnd_Y,Plate_H-Wire_d/2]) rotate([-90,0,0]) cylinder(d=Wire_d+IDXtra*2, h=Len);
		} // hull
		
		//translate([0,ControlRod_Y,-Overlap]) cylinder(d=7, h=50);
	} // difference
	
} // WireBendingJig

// WireBendingJig();

module TestFixtureAlpha(Baseboard_t=15){
	Fixture_X=38;
	Fixture_Y=46;
	Fixture_t=Baseboard_t;
	Len=46;
	ControlRodEnd_Y=-Len/2+2.5;
	ControlRod_Y=Len/2-5;
	MountingPlate_Y=Fixture_Y+20;
	MountingHole_bs=Len+10;
	
	Tie_Len=30.4;
	
	difference(){
		union(){
			RoundRect(X=Fixture_X, Y=MountingPlate_Y, Z=Fixture_t, R=2);
			
			translate([Tie_Len/2+2, MountingPlate_Y/2-5, 0]) RoundRect(X=4, Y=10, Z=Fixture_t+2);
			translate([Tie_Len/2+2, MountingPlate_Y/2-25, 0]) RoundRect(X=4, Y=10, Z=Fixture_t+2);
			translate([Tie_Len/2-32-2, MountingPlate_Y/2-25, 0]) RoundRect(X=4, Y=10, Z=Fixture_t+2);
			translate([-Tie_Len/2-2, MountingPlate_Y/2-5, 0]) RoundRect(X=4, Y=10, Z=Fixture_t+2);
		} // union
		
		// Wire hole
		translate([0,ControlRod_Y,-Overlap]) cylinder(d=7, h=50);
		
		translate([0,MountingHole_bs/2,Fixture_t]) Bolt6Hole();
		translate([0,-MountingHole_bs/2,Fixture_t]) Bolt6Hole();

	} // difference
} // TestFixtureAlpha

// TestFixtureAlpha();

module ServoHornExtension(){
	HornCollar_d=7;
	H=10;
	Pusher_Y=7;
	Collar_H=2.7;
	TorquePost_Y=-5;
	HornGrip_H=1.8;
	
	difference(){
		union(){
			hull(){
				cylinder(d=HornCollar_d+3+IDXtra*2, h=Collar_H);
				translate([0,Pusher_Y,0]) scale([1,0.6,1]) cylinder(d=9, h=Collar_H);
				translate([0,TorquePost_Y,0]) scale([1,0.6,1]) cylinder(d=9, h=Collar_H);
			} // hull
			
			// Wire pusher
			hull(){
				translate([3.4,Pusher_Y,0]) scale([0.7,1,1]) cylinder(d=4, h=H);
				translate([-3.4,Pusher_Y,0]) scale([0.7,1,1]) cylinder(d=4, h=H);
			} // hull
			
			// Horn gripper
			hull(){
				translate([3.4,TorquePost_Y,0]) scale([0.7,1,1]) cylinder(d=4, h=Collar_H+HornGrip_H);
				translate([-3.4,TorquePost_Y,0]) scale([0.7,1,1]) cylinder(d=4, h=Collar_H+HornGrip_H);
			} // hull
		} // union
		
		// Servo horn clearance
		translate([0,0,Collar_H]) hull(){
			cylinder(d=7, h=HornGrip_H+Overlap);
			translate([0,-14,0]) cylinder(d=4, h=HornGrip_H+Overlap);
		} // hull
		
		translate([0,0,-Overlap]) cylinder(d=HornCollar_d+IDXtra*2, h=3+Overlap*2);
		
		// Passage
		Passage_H=4;
		Passage_W=1.5;
		translate([0,Pusher_Y,H-Passage_H])
			difference(){
				cylinder(d=10, h=Passage_H);
				
				hull(){
					translate([Passage_W/2+1.5,0,-Overlap]) cylinder(d=3,h=Passage_H+Overlap*2);
					translate([Passage_W/2+1.5+5,5,-Overlap]) cylinder(d=3,h=Passage_H+Overlap*2);
					translate([Passage_W/2+1.5+5,-5,-Overlap]) cylinder(d=3,h=Passage_H+Overlap*2);
				} // hull
				
				hull(){
					translate([-Passage_W/2-1.5,0,-Overlap]) cylinder(d=3,h=Passage_H+Overlap*2);
					translate([-Passage_W/2-1.5-5,5,-Overlap]) cylinder(d=3,h=Passage_H+Overlap*2);
					translate([-Passage_W/2-1.5-5,-5,-Overlap]) cylinder(d=3,h=Passage_H+Overlap*2);
				} // hull
			} // difference
			
	} // difference
	
} // ServoHornExtension

// ServoHornExtension();
































