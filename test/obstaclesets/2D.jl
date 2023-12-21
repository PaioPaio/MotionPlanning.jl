using MotionPlanning

ISRR_2H(::Type{T}=Float64) where {T<:AbstractFloat} = Compound2D(
    Shape2D{T}[
        Box2D([T(0.0), T(0.16)], [T(0.36), T(0.5)]),
        Box2D([T(0.4), T(0.5)], [T(0.19), T(0.35)]),
        Box2D([T(0.22), T(0.46)], [T(0.57), T(0.75)]),
        Box2D([T(0.75), T(1.0)], [T(0.64), T(0.77)]),
        Box2D([T(0.22), T(0.8)], [T(0.34), T(0.51)])
    ]
)

TRI_BALLS(::Type{T}=Float64) where {T<:AbstractFloat} = Compound2D(
    Shape2D{T}[
        Polygon([(T(0.3), T(0.3)), (T(0.7), T(0.3)), (T(0.5), T(0.65))]),
        Circle([T(0.3), T(0.3)], T(0.15)),
        Circle([T(0.7), T(0.3)], T(0.15)),
        Circle([T(0.5), T(0.65)], T(0.15))
    ]
)

ISRR_POLY(::Type{T}=Float64) where {T<:AbstractFloat} = Compound2D(
    Shape2D{T}[
        Polygon([(T(0.0), T(0.25)), (T(0.27), T(0.28)), (T(0.17), T(0.4)), (T(0.0), T(0.4))]),
        Polygon([(T(0.5), T(0.2)), (T(0.2), T(0.5)), (T(0.25), T(0.7)), (T(0.4), T(0.8)), (T(0.6), T(0.8)), (T(0.7), T(0.5))]),
        Polygon([(T(0.55), T(0.2)), (T(0.75), T(0.5)), (T(0.85), T(0.5)), (T(0.85), T(0.2))]),
        # Polygon([(T(.3), T(.6)), (T(.15), T(.85)), (T(.4), T(.6))]),
        Circle([T(0.9), T(0.65)], T(0.1))
    ]
)

ISRR_POLY_WITH_SPIKE(::Type{T}=Float64) where {T<:AbstractFloat} = Compound2D(
    Shape2D{T}[
        Polygon([(T(0.0), T(0.25)), (T(0.27), T(0.28)), (T(0.17), T(0.4)), (T(0.0), T(0.4))]),
        Polygon([(T(0.5), T(0.2)), (T(0.2), T(0.5)), (T(0.25), T(0.7)), (T(0.4), T(0.8)), (T(0.6), T(0.8)), (T(0.7), T(0.5))]),
        Polygon([(T(0.55), T(0.2)), (T(0.75), T(0.5)), (T(0.85), T(0.5)), (T(0.85), T(0.2))]),
        Polygon([(T(0.3), T(0.6)), (T(0.15), T(0.85)), (T(0.4), T(0.6))]),
        Circle([T(0.9), T(0.65)], T(0.1))
    ]
)

EMPTY_2D{T<:AbstractFloat}(::Type{T}=Float64) = Compound2D(Shape2D{T}[])

nothing