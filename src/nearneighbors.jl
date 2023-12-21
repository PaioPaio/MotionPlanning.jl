export Nearest, NeighborInfo, Neighborhood, PackedNeighborhoods
export NearNeighborDataStructure, MetricNNDS, NullNNDS, MemoizedNNDS, use_NullNNDS, use_MemoizedNNDS
export NeighborhoodCache, StreamingNC, MemoizedNC, OfflineNC
export includes_controls, neighbor_info_type, neighbors

# Utilities
const F_or_B = Union{Val{:F},Val{:B}}
const BoolVal = Union{Val{true},Val{false}}
@inline keep_controls_if(nt::NamedTuple, cond::BoolVal) = cond === Val(true) ? nt : (nt..., controls=missing)
# @inline lattice_neighbors(v::SVector{D}) where {D} = (setindex(v, v[i] + j, i) for i in 1:D for j in (-1, 1))
@inline lattice_neighbors(v::SVector{D}) where {D} = (v + SVector(Tuple(I)) - SVector(ntuple(i -> 2, D)) for
                                                      I in CartesianIndices(ntuple(i -> 3, D)) if
                                                      I != CartesianIndex(ntuple(i -> 2, D)))        # JuliaLang/julia#29440
@inline includes_controls(x) = includes_controls(typeof(x))
@inline neighbor_info_type(x) = neighbor_info_type(typeof(x))

# Neighborhoods
const NeighborInfo{X<:SampleIndex,D,U} = NamedTuple{(:index, :cost, :controls),Tuple{X,D,U}}
const Neighborhood{X<:SampleIndex,D,U} = AbstractVector{NeighborInfo{X,D,U}}
includes_controls(::Type{NeighborInfo{X,D,U}}) where {X,D,U} = U !== Missing
includes_controls(::Type{<:Neighborhood{X,D,U}}) where {X,D,U} = U !== Missing
neighbor_info_type(::Type{<:Neighborhood{X,D,U}}) where {X,D,U} = NeighborInfo{X,D,U}
indextype(::Type{NeighborInfo{X,D,U}}) where {X,D,U} = X
costtype(::Type{NeighborInfo{X,D,U}}) where {X,D,U} = D
controlstype(::Type{NeighborInfo{X,D,U}}) where {X,D,U} = U
function _neighbor_info_type(::Type{X}, ::Type{NamedTuple{M,Tuple{D,U}}}, include_controls::BoolVal) where {X,M,D,U}
    include_controls === Val(true) ? NeighborInfo{X,D,U} : NeighborInfo{X,D,Missing}
end
function neighbor_info_type(nodes::SampleSet, bvp; include_controls::BoolVal=Val(false))
    X = indextype(nodes)
    BVP = typeof(bvp(nodes.init, nodes.init))
    _neighbor_info_type(X, BVP, include_controls)
end
function shift_lattice_component(n::NeighborInfo{<:TiledIndex}, shift)
    (index=shift_lattice_component(n.index, shift), cost=n.cost, controls=n.controls)
end
function neighbor_info_iterator(nodes, bvp, x, inds, r=nothing; dir::F_or_B=Val(:F),
    include_controls::BoolVal=Val(false), omit::F=always_false) where {F}
    ((index=i, keep_controls_if(dir === Val(:F) ? (r === nothing ? bvp(x, nodes[i]) : bvp(x, nodes[i], r)) :
                                (r === nothing ? bvp(nodes[i], x) : bvp(nodes[i], x, r)),
        include_controls)...) for i in inds if !omit(i))
end
function reverse_neighbor_dict(i, neighborhood, ::Type{NeighborInfo{X,D,U}}) where {X,D,U}
    Dict{X,NeighborInfo{X,D,U}}(n.index => (index=i, cost=n.cost, controls=n.controls) for n in neighborhood)
end

## CSC-style Neighborhoods
struct PackedNeighborhoods{X,D,U,N<:Neighborhood{X,D,U}} <: AbstractVector{N}
    ptr::Vector{Int}
    val::Vector{NeighborInfo{X,D,U}}
    neighborhoods::Vector{N}
end
function PackedNeighborhoods(neighborhoods::AbstractVector{<:Neighborhood})
    ptr = pushfirst!(cumsum([length(n) for n in neighborhoods]) + 1, 1)
    val = reduce(vcat, neighborhoods)
    PackedNeighborhoods(ptr, val, [view(val, ptr[i]:ptr[i+1]-1) for i in 1:length(neighborhoods)])
end
Base.length(PN::PackedNeighborhoods) = length(PN.neighborhoods)
Base.getindex(PN::PackedNeighborhoods, i::Int) = PN.neighborhoods[i]
includes_controls(::Type{<:PackedNeighborhoods{X,D,U}}) where {X,D,U} = U !== Missing
neighbor_info_type(::Type{<:PackedNeighborhoods{X,D,U}}) where {X,D,U} = NeighborInfo{X,D,U}

# k-Nearest Neighbors
struct Nearest
    k::Int
end
placeholder_neighborhood(::Type{T}, k::Int) where {T<:NeighborInfo} = Vector{Union{Missing,T}}(missing, k)
function update_knn!(neighborhood, n::NeighborInfo{X,D,U}) where {X,D,U}
    if neighborhood[1] === missing || n.cost < neighborhood[1].cost
        neighborhood[1] = n
        DataStructures.percolate_down!(neighborhood, 1, Base.Order.By(x -> x === missing ? D(-Inf) : -x.cost))
    end
    neighborhood
end

# Near Neighborhood Caches
abstract type NeighborhoodCache end
includes_controls(::Type{<:NeighborhoodCache}) = false
function neighbors(cache::NeighborhoodCache, nnds, nodes, bvp, v_or_x, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false))
    neighbors(nnds, nodes, bvp, v_or_x, r, dir=dir, include_controls=include_controls)
end

## StreamingNC
struct StreamingNC <: NeighborhoodCache end
reset!(::StreamingNC) = nothing
addstates!(::StreamingNC, X) = nothing

## MemoizedNC
struct MemoizedNC{N<:Neighborhood} <: NeighborhoodCache
    neighborhoods_F::Vector{Union{Missing,N}}
    neighborhoods_B::Vector{Union{Missing,N}}
end
function MemoizedNC(::Type{T}, n::Int=0, issymmetric::Bool=false) where {T}
    if issymmetric
        neighborhoods = Vector{Union{Missing,Vector{T}}}(missing, n)
        MemoizedNC(neighborhoods, neighborhoods)
    else
        MemoizedNC(Vector{Union{Missing,Vector{T}}}(missing, n), Vector{Union{Missing,Vector{T}}}(missing, n))
    end
end
function MemoizedNC(nodes::SampleSet, bvp::SteeringBVP; include_controls::BoolVal=Val(false))
    MemoizedNC(neighbor_info_type(nodes, bvp, include_controls=include_controls),
        length(nodes), issymmetric(bvp) && include_controls === Val(false))
end
includes_controls(::Type{MemoizedNC{N}}) where {N} = includes_controls(N)
neighbor_info_type(::Type{MemoizedNC{N}}) where {N} = neighbor_info_type(N)
reset!(cache::MemoizedNC) = (fill!(cache.neighborhoods_F, missing); fill!(cache.neighborhoods_B, missing))
function neighbors(cache::MemoizedNC{N}, nnds, nodes, bvp, v::SampleIndex, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false)) where {N}
    neighborhoods = dir === Val(:F) ? cache.neighborhoods_F : cache.neighborhoods_B
    i = linear_index(v)
    neighborhood = i > 0 ? neighborhoods[i] : missing
    if neighborhood === missing
        neighborhood = collect_if_not_Array(neighbors(nnds, nodes, bvp, v, r,
            dir=dir, include_controls=include_controls))
        i > 0 && (neighborhoods[i] = neighborhood)
    end
    neighborhood
end

## OfflineNC
struct OfflineNC{PN<:PackedNeighborhoods} <: NeighborhoodCache
    neighborhoods_F::PN
    neighborhoods_B::PN
end
function OfflineNC(cache::MemoizedNC)
    if cache.neighborhoods_F === cache.neighborhoods_B
        packed_neighborhoods = PackedNeighborhoods(cache.neighborhoods_F)
        OfflineNC(packed_neighborhoods, packed_neighborhoods)
    else
        OfflineNC(PackedNeighborhoods(cache.neighborhoods_F), PackedNeighborhoods(cache.neighborhoods_B))
    end
end
includes_controls(::Type{OfflineNC{PN}}) where {PN} = includes_controls(PN)
neighbor_info_type(::Type{OfflineNC{PN}}) where {PN} = neighbor_info_type(PN)
function neighbors(cache::OfflineNC{PN}, nnds, nodes, bvp, v::SampleIndex, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false)) where {PN}
    dir === Val(:F) ? cache.neighborhoods_F[linear_index(v)] : cache.neighborhoods_B[linear_index(v)]
end

# Near Neighbor Data Structures
abstract type NearNeighborDataStructure end
abstract type MetricNNDS <: NearNeighborDataStructure end    # `import NearestNeighbors` or `import FLANN`
includes_controls(::Type{<:NearNeighborDataStructure}) = false
## NullNNDS (Streaming)
struct NullNNDS <: NearNeighborDataStructure end
NullNNDS(nodes::SampleSet, bvp::SteeringBVP; kwargs...) = NullNNDS()
use_NullNNDS() = (default_NN_data_structure_type[] = :NullNNDS)
reset!(::NullNNDS) = nothing
addstates!(::NullNNDS, X) = nothing
function neighbors(nnds::NullNNDS, nodes, bvp, v_or_x, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false))
    neighbors(nodes, bvp, v_or_x, r, dir=dir, include_controls=include_controls)
end

## MemoizedNNDS
struct MemoizedNNDS{S,D,U,F} <: NearNeighborDataStructure
    bvp_results::Dict{Tuple{S,S},NamedTuple{(:cost, :controls),Tuple{D,U}}}
    memoized_bvp::F
end
function MemoizedNNDS(nodes::ExplicitSampleSet, bvp::SteeringBVP; include_controls::BoolVal=Val(false))
    S = typeof(nodes.init)
    D = typeof(bvp(nodes.init, nodes.init).cost)
    U = include_controls === Val(true) ? typeof(bvp(nodes.init, nodes.init).controls) : Missing
    bvp_results = Dict{Tuple{S,S},NamedTuple{(:cost, :controls),Tuple{D,U}}}()
    memoized_bvp = (x0, xf, cost_bound=D(Inf)) -> begin    # TODO: check if closure performance is disastrous
        get!(bvp_results, (x0, xf)) do                     #       also, make subtype of SteeringBVP?
            keep_controls_if(bvp(x0, xf, cost_bound), include_controls)
        end
    end
    MemoizedNNDS(bvp_results, memoized_bvp)
end
use_MemoizedNNDS() = (default_NN_data_structure_type[] = :MemoizedNNDS)
includes_controls(::MemoizedNNDS{S,D,U}) where {S,D,U} = U !== Missing
reset!(nnds::MemoizedNNDS) = empty!(nnds.bvp_results)
addstates!(::MemoizedNNDS, X) = nothing
function neighbors(nnds::MemoizedNNDS, nodes, bvp, v_or_x, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false))
    neighbors(nodes, nnds.memoized_bvp, v_or_x, r, dir=dir, include_controls=include_controls)
end

# General Methods
function neighbors(nodes::ExplicitSampleSet, bvp, v::Int, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false))
    neighbors(nodes, bvp, nodes[v], r, dir=dir, include_controls=include_controls, omit=isequal(v))
end
function neighbors(nodes::ExplicitSampleSet, bvp, x::State, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false), omit::F=always_false) where {F}
    itr = neighbor_info_iterator(nodes, bvp, x, eachindex(nodes), r,
        dir=dir, include_controls=include_controls, omit=omit)
    Iterators.filter(n -> n.cost <= r, itr)
end
# If r = Nearest, then r=missing in neighbor_info_iterator
function neighbors(nodes::ExplicitSampleSet, bvp, x::State, r::Nearest;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false), omit::F=always_false) where {F}
    itr = neighbor_info_iterator(nodes, bvp, x, eachindex(nodes), dir=dir, include_controls=include_controls, omit=omit)
    neighborhood = placeholder_neighborhood(neighbor_info_type(nodes, bvp, include_controls=include_controls), r.k)
    foreach(n -> update_knn!(neighborhood, n), itr)
    neighborhood
end

function neighbors(nodes::TiledSampleSet, bvp, v::TiledIndex, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false))
    if v.off_lattice <= 0
        neighbors(nodes, bvp, nodes[v], r, dir=dir, include_controls=include_controls)
    else
        if dir === Val(:F)
            x0_xf = i -> (nodes[zero_lattice_component(v)], nodes[shift_lattice_component(i, -v.on_lattice)])
        else
            x0_xf = i -> (nodes[zero_lattice_component(i)], nodes[shift_lattice_component(v, -i.on_lattice)])
        end
        neighbors(nodes, bvp, nodes[v], r, dir=dir, include_controls=include_controls, omit=isequal(v), x0_xf=x0_xf)
    end
end
function neighbors(nodes::TiledSampleSet, bvp, x::State, r;
    dir::F_or_B=Val(:F), include_controls::BoolVal=Val(false), omit::F=always_false,
    x0_xf=(dir === Val(:F) ? i -> (x, nodes[i]) : i -> (nodes[i], x))) where {F}
    neighborhood = neighbor_info_type(nodes, bvp, include_controls=include_controls)[]
    close_idx = findclose(nodes, bvp, x, r, dir=dir, omit=omit, x0_xf=x0_xf)
    queue = [close_idx.on_lattice]
    visited = Set(queue)
    while !isempty(queue)
        on_lattice_idx = popfirst!(queue)
        found_a_neighbor = false
        for i in eachindex(nodes.Voff)
            idx = TiledIndex(on_lattice_idx, i)
            n = (index=idx, bvp(x0_xf(idx)..., r)...)
            if n.cost <= r
                found_a_neighbor = true
                !omit(idx) && push!(neighborhood, keep_controls_if(n, include_controls))
            end
        end
        found_a_neighbor && for oli in lattice_neighbors(on_lattice_idx)
            if !(oli in visited)
                push!(queue, oli)
                push!(visited, oli)
            end
        end
    end
    neighborhood
end
function findclose(nodes::TiledSampleSet, bvp, x::State, r; dir::F_or_B=Val(:F), omit::F=always_false,
    x0_xf=(dir === Val(:F) ? i -> (x, nodes[i]) : i -> (nodes[i], x)), cutoff::Int=30) where {F}
    Q, R = qr(nodes.A)
    start_lattice_idx = round.(Int, R \ Q' * (x - nodes.b))
    sentinel = fill(typemin(Int), typeof(start_lattice_idx))
    queue = [start_lattice_idx, sentinel]
    visited = Set(queue)
    ct = 0
    while !isempty(queue)
        on_lattice_idx = popfirst!(queue)
        if on_lattice_idx == sentinel
            push!(queue, sentinel)
            ct += 1
            ct > cutoff && error("Either r is too small, or searching near the starting " *
                                 "state isn't working to find a close tiled sample!")
            continue
        end
        for i in eachindex(nodes.Voff)
            idx = TiledIndex(on_lattice_idx, i)
            omit(idx) && continue
            bvp(x0_xf(idx)..., r).cost <= r && return idx
        end
        for oli in lattice_neighbors(on_lattice_idx)
            if !(oli in visited)
                push!(queue, oli)
                push!(visited, oli)
            end
        end
    end
end
