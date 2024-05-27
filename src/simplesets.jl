export SimpleConvexSet, AxisAlignedBox, Ball, SetComplement
export inflate, boundingbox, dimension, volume

abstract type SimpleConvexSet end

struct AxisAlignedBox{N,T} <: SimpleConvexSet
    lo::SVector{N,T}
    hi::SVector{N,T}
end
AxisAlignedBox(lo, hi) = AxisAlignedBox{length(lo),eltype(lo)}(lo, hi)
Base.in(x, B::AxisAlignedBox{N}) where {N} = all(B.lo[i] <= x[i] <= B.hi[i] for i in 1:N)
Base.rand(B::AxisAlignedBox) = B.lo + (B.hi - B.lo).*rand(typeof(B.lo))
inflate(B::AxisAlignedBox, ε) = AxisAlignedBox(B.lo .- ε, B.hi .+ ε)
dimension(B::AxisAlignedBox{N}) where {N} = N
boundingbox(B::AxisAlignedBox) = B
volume(B::AxisAlignedBox) = prod(B.hi - B.lo)

struct Ball{N,T} <: SimpleConvexSet
    c::SVector{N,T}
    r::T
end
Ball(c, r) = Ball{length(c),eltype(c)}(c, r)
Base.in(x, B::Ball{N}) where {N} = norm(x - B.c) <= B.r
Base.rand(B::Ball) = (x = B.c + 2*B.r*(rand(typeof(B.c)) .- 1//2); x in B ? x : rand(B))
inflate(B::Ball, ε) = Ball(B.c, B.r + ε)
dimension(B::Ball{N}) where {N} = N
boundingbox(B::Ball) = AxisAlignedBox(B.c .- B.r, B.c .+ B.r)
volume(::Type{Ball{N,T}}) where {N,T} = (k = div(N, 2); iseven(N) ? T(π)^k/factorial(k) :
                                                                    2*factorial(k)*(4*T(π))^k/factorial(N))
volume(::Type{Ball{N}}) where {N} = volume(Ball{N,Float64})
volume(B::Ball{N,T}) where {N,T} = volume(Ball{N,T})*B.r^N

struct SetComplement{S}
    set::S
end
SetComplement(S::SetComplement) = S.set
Base.in(x, S::SetComplement) = !(x in S.set)
