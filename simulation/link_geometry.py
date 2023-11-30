rgba=(
    (0.0, 1.0, 0.0,),
    (1.0, 0.5, 0.0,),
    (0.0, 0.0, 1.0,),
    (1.0, 0.0, 1.0,),
    (1.0, 1.0, 0.0,),
    (1.0, 0.0, 0.0,),
)
vertices, indices = geometry.create_cube(scale=(2.0,2.0,2.0), rgba=rgba, dtype='float32')
vertices = vertices[indices]
vertices.dtype = [
    ('position',    'float32',  (3,)),
    ('colour',      'float32',  (3,)),
]