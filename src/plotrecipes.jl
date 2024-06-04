# Problem (main)
@recipe function f(P::MPProblem; dims=(1, 2),
    statespace_color=:black, statespace_alpha=1,
    init_color=:coral4, init_alpha=1, init_markershape=:diamond, init_markersize=5,
    goal_color=:green, goal_alpha=1, goal_markershape=:star, goal_markersize=10,
    obstacle_color=:red, obstacle_alpha=1,
    show_graph=false, graph_color=:grey, graph_alpha=0.5, graph_markersize=2, graph_edge_waypoints=10,
    solution_color=:blue, solution_alpha=1, solution_markersize=2, solution_linewidth=2, solution_edge_waypoints=10,
    annotationstext=nothing)
    dims --> dims
    label --> ""
    x, y = dims
    @series begin
        color --> statespace_color
        alpha --> statespace_alpha
        P.state_space
    end
    @series begin
        color --> obstacle_color
        alpha --> obstacle_alpha
        P.collision_checker
    end
    if show_graph && :graph in keys(P.solution.metadata)
        @series begin
            color_edge --> graph_color
            alpha --> graph_alpha
            markersize --> graph_markersize
            num_waypoints --> graph_edge_waypoints
            state2config --> P.collision_checker.state2config
            labelstext --> annotationstext
            # config2viz     --> P.collision_checker.config2viz    # TODO
            plot_endpoints --> true
            plot_x0 --> false
            plot_xf --> true
            graph = P.solution.metadata[:graph]
            graph isa ForwardAdjacencyDict ? [P.graph[i, j] for (i, js) in P.solution.metadata[:graph].d for j in js] :
            [P.graph[i, j] for (j, is) in P.solution.metadata[:graph].d for i in is]
        end
    end
    if P.status === :solved
        @series begin
            color --> solution_color
            alpha --> solution_alpha
            markersize --> solution_markersize
            linewidth --> solution_linewidth
            num_waypoints --> solution_edge_waypoints
            state2config --> P.collision_checker.state2config
            # config2viz     --> P.collision_checker.config2viz    # TODO
            plot_endpoints --> true
            [P.graph[i, j] for (i, j) in pairwise(P.solution.metadata[:solution_nodes])]
        end
    end
    @series begin
        color --> goal_color
        alpha --> goal_alpha
        markershape --> goal_markershape
        markersize --> goal_markersize
        P.goal
    end
    @series begin
        color --> init_color
        alpha --> init_alpha
        markershape --> init_markershape
        markersize --> init_markersize
        [P.init[1]], [P.init[2]]
    end
end

# Edges

@recipe function f(e::SteeringEdge; state2config=identity, config2viz=identity,
    dims=(1, 2), num_waypoints=10, plot_endpoints=true,
    plot_x0=true, plot_xf=true)
    state2config --> state2config
    config2viz --> config2viz
    dims --> dims
    num_waypoints --> num_waypoints
    plot_endpoints --> plot_endpoints
    plot_x0 --> plot_x0
    plot_xf --> plot_xf
    SVector(e)
end

@recipe function f(edges::AbstractVector{<:SteeringEdge}; state2config=identity, config2viz=identity,
    dims=(1, 2), num_waypoints=10, plot_endpoints=true,
    labelstext=nothing,
    plot_x0=false, plot_xf=true, color_edge=:grey)
    x, y = dims
    X = Union{Float64,Vector{Float64}}[]
    Y = Union{Float64,Vector{Float64}}[]
    for e in edges
        pts = waypoints(e, num_waypoints)
        append!(X, [[p[x] for p in pts]])
        append!(Y, [[p[y] for p in pts]])
    end
    for i in eachindex(edges)
        @series begin
            seriestype := :line
            if color_edge isa Array
                color --> color_edge[i]
            else
                color --> color_edge
            end
            X[i], Y[i]
        end
    end
    plot_endpoints && plot_x0 && @series begin
            seriestype := :scatter
            color --> color_edge
            pts = [state2config(e.x0) for e in edges]
            [p[x] for p in pts], [p[y] for p in pts]
        end
    plot_endpoints && plot_xf && @series begin
            seriestype := :scatter
            series_annotations --> labelstext
            color --> color_edge
            pts = [state2config(e.xf) for e in edges]
            [p[x] for p in pts], [p[y] for p in pts]
        end
end

@recipe function f(SS::BoundedStateSpace; dims=(1, 2)) #, statespace_color=:black, statespace_linecolor=statespace_color)
    dims --> dims
    linecolor := :match
    fillalpha := 0
    label --> ""
    SS.bounded_set
end

@recipe function f(CC::CollisionChecker; dims=(1, 2))
    dims --> dims
    foreach(o -> @series(begin
        o
    end), CC.obstacles)
end

@recipe function f(G::ConfigSpaceGoal; dims=(1, 2))
    dims --> dims
    G.projection
end

@recipe function f(G::StateSpaceGoal; dims=(1, 2))
    if G.set isa AbstractArray{<:State}
        seriestype --> :scatter
        markercolor := :match
        x, y = dims
        [q[x] for q in G.set], [q[y] for q in G.set]
    else
        dims --> dims
        G.set
    end
end

@recipe function f(o::Obstacle; dims=(1, 2))
    dims --> dims
    o.set
end

@recipe function f(B::AxisAlignedBox; dims=(1, 2))
    seriestype := :shape
    fillcolor --> :match
    linecolor --> :match
    label --> ""
    x, y = dims
    SVector(B.lo[x], B.hi[x], B.hi[x], B.lo[x]), SVector(B.lo[y], B.lo[y], B.hi[y], B.hi[y])
end

@recipe function f(B::Ball; dims=(1, 2), n=64)
    seriestype := :shape
    fillcolor --> :match
    linecolor --> :match
    label --> ""
    x, y = dims
    θ = range(-π, stop=π, length=n)
    B.c[x] .+ B.r * cos.(θ), B.c[y] .+ B.r * sin.(θ)
end

@recipe f(S::SetComplement; dims=(1, 2)) = nothing