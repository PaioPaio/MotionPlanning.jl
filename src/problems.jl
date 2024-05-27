export MPProblem, MPSolution

struct MPSolution{T}
    status::Symbol
    cost::T
    elapsed::Float64
    metadata::Dict{Symbol,Any}
end

mutable struct MPProblem
    state_space::StateSpace
    bvp::SteeringBVP
    init::State
    goal::Goal
    collision_checker::CollisionChecker
    status::Symbol
    graph::NearNeighborGraph
    solution::MPSolution

    function MPProblem(state_space::StateSpace,
        bvp::SteeringBVP,
        init::State,
        goal::Goal,
        collision_checker::CollisionChecker)
        new(state_space, bvp, init, goal, collision_checker, :unsolved)
    end
end
