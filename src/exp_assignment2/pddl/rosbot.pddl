;The domain file (rosbot.pdll) encapsulates the core components of a planning scenario, 
;encompassing aspects like object types, their interconnections, and the feasible actions.
;Paired with a related problem file (exploration.pdll), the domain file serves as input 
;for a PDDL planner, enabling the generation of a plan aimed at accomplishing a predefined
;objective within the established domain.

;Header and description
(define (domain rosbot)

;remove requirements that are not needed
(:requirements 
	:strips 
	:fluents 
	:typing 
	:disjunctive-preconditions 
	:durative-actions)

;------------------------------Types------------------------------
; 1. waypoint: Represents a location in the environment where the robot can navigate or operate.
; 2. marker:   Denotes an identifiable object or point of interest in the environment.
; 3. station:  Refers to a fixed location that serves as a designated point within the environment.

(:types 
    waypoint
    marker
    station
)

;------------------------------Predicates------------------------------
; 1. (robot_at ?x - (either station waypoint)):
;    Represents the current location of the robot, which can be either a station or a waypoint.
;
; 2. (detected ?mk - marker):
;    Signifies that the specified marker (?mk) has been successfully detected by the robot.
;
; 3. (visible ?mk - marker ?wp - waypoint):
;    Indicates that the marker (?mk) is visible from the given waypoint (?wp),
;    providing information about the robot's line of sight to the marker.

(:predicates 
    (robot_at ?x - (either station waypoint))
    (detected ?mk - marker)
    (visible ?mk - marker ?wp - waypoint)
)

;------------------------------Actions------------------------------
;Initiates a mission from a specified station to a waypoint.
(:durative-action start_mission 
    :parameters (?from - station ?to - waypoint)
    :duration( = ?duration 30)
    :condition (at start (robot_at ?from))
    :effect (and (at end (robot_at ?to)) (at start (not(robot_at ?from))))
)

;Represents the movement of the robot from one waypoint to another.
(:durative-action move
    :parameters (?from ?to - waypoint)
    :duration( = ?duration 30)
    :condition (at start (robot_at ?from))
    :effect (and (at end (robot_at ?to)) (at start (not(robot_at ?from))))
)

;Represents the robot's task of locating a specific marker from a given waypoint.
(:durative-action find_marker
    :parameters (?wp - waypoint ?mk - marker)
    :duration( = ?duration 10)
    :condition (and (over all (robot_at ?wp)) (over all (visible ?mk ?wp)))
    :effect (at end (detected ?mk))
)

;Represents the conclusion of a mission, where the robot moves from a waypoint to a specified station.
(:durative-action end_mission
    :parameters (?from - waypoint ?to - station)
    :duration( = ?duration 30)
    :condition (at start (robot_at ?from))
    :effect (and (at end (robot_at ?to)) (at start (not(robot_at ?from))))
)

)
