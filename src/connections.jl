export Pairwise, pairwise, collision_waypoints_itr
export SteeringEdge

# Pairwise Iterator
struct Pairwise{I}
    itr::I
end
pairwise(itr) = Pairwise(itr)
Base.length(p::Pairwise) = length(p.itr) - 1
Base.eltype(::Type{Pairwise{I}}) where {I} = Tuple{eltype(I),eltype(I)}
@inline function Base.iterate(p::Pairwise, state=iterate(p.itr))
    state === nothing && return nothing
    first_item, itr_state = state
    state = iterate(p.itr, itr_state)
    state === nothing && return nothing
    (first_item, state[1]), state
end

# Steering Connection
struct SteeringEdge{BVP,S,T,U}
    bvp::BVP
    x0::S
    xf::S
    cost::T
    controls::U
end
function SteeringEdge(bvp::BVP, x0::S, xf::S, cost=missing, controls::Missing=missing) where {BVP,S}
    SteeringEdge(bvp, x0, xf, bvp(x0, xf)...)
end
DifferentialDynamicsModels.waypoints(e::SteeringEdge, dt_or_N) = waypoints(e.bvp.dynamics, e.x0, e.controls, dt_or_N)

# SteeringBVP Collision Checking
collision_waypoints_itr(f::DifferentialDynamics, x::State, controls) = waypoints_itr(f, x, controls, 8)
function is_free_edge(CC::CollisionChecker, bvp::SteeringBVP, x0::State, xf::State, controls::Missing=missing)
    is_free_edge(CC, bvp.dynamics, x0, bvp(x0, xf).controls)
end
function is_free_edge(CC::CollisionChecker, bvp::SteeringBVP, x0::State, xf::State, controls)
    is_free_edge(CC, bvp.dynamics, x0, controls)
end
function is_free_edge(CC::CollisionChecker, f::DifferentialDynamics, x0::State, controls)
    CC.edge_count[] += 1
    all(is_free_motion(CC, x1, x2) for (x1, x2) in pairwise(collision_waypoints_itr(f, x0, controls)))
end
## Geometric Steering
function is_free_edge(CC::CollisionChecker, bvp::GeometricSteering, x0::State, xf::State, controls::Missing=missing)
    CC.edge_count[] += 1
    is_free_motion(CC, x0, xf)
end
function is_free_edge(CC::CollisionChecker, bvp::GeometricSteering, x0::State, xf::State, controls)
    CC.edge_count[] += 1
    is_free_motion(CC, x0, xf)
end

# Partial Steering (for RRT and variants)
## Time -- includes GeometricSteering, TODO: bring back cost_waypoint? or quantify steering limit by duration?
function steer_towards(bvp::SteeringBVP{<:Any,Time}, x0, xf, r, cost=missing, controls::Missing=missing)
    steer_towards(bvp, x0, xf, r, bvp(x0, xf)...)
end
function steer_towards(bvp::SteeringBVP{<:Any,Time}, x0, xf, r, cost, controls)
    if r < cost
        xf = propagate(bvp.dynamics, x0, controls, r)
        (xf=xf, bvp(x0, xf)...)    # TODO: splitcontrol
    else
        (xf=xf, cost=cost, controls=controls)
    end
end
