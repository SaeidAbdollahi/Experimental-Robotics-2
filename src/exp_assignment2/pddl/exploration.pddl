;The PDDL problem file (exploration.pddl), together with its associated domain file (rosbot.pddl),
;acts as input for a PDDL planner. Utilizing these files, the planner generates a plan aimed at
;transitioning from the initial state to the goal state within the specified planning domain.

;Objects of the world
(define (problem exploration) (:domain rosbot)
(:objects 
    wp0 - station
    wp1 wp2 wp3 wp4 - waypoint
    mk11 mk12 mk13 mk15 - marker
)

;The initial state of the world
(:init
    (robot_at wp0)
    (visible mk11 wp1) (visible mk12 wp2) (visible mk13 wp3) (visible mk15 wp4)
    
    )

;The goal state of the world
(:goal (and
    (detected mk11) (detected mk12) (detected mk13) (detected mk15)
    (robot_at wp0)
))
)

