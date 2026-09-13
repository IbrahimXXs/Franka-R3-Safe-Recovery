# Geometry correction

The upstream Python config said 8.1 mm bore; the actual USD collision mesh is
9.0 mm (144-sided straight wall, 1 mm entry chamfer). The peg diameter is
7.986 mm, with 0.5 mm end chamfers. Vertex-radius radial clearance is 0.507 mm;
conservative wall-inradius clearance is 0.5059 mm. Previous 0.057 mm metadata
and its 0.01425 mm numerical screen were based on stale configuration metadata.
Raw forces, poses and sensor transforms are unchanged. The zero-contact baseline
and its clearances still pass the corrected screen. These files retain their
original settings; use the corrected protocol for new contact characterization.
