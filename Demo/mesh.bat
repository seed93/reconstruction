PoissonRecon.x64.exe --in tmp\cloud_filter.ply --out tmp\mesh.ply --depth 9 --density --samplesPerNode 2 --pointWeight 0 --solverDivide 9
SurfaceTrimmer.x64.exe --in tmp\mesh.ply --out tmp\mesh_trimmer.ply --smooth 100 --trim 7 --aRatio 0.01
meshlab\\meshlabserver.exe -i tmp\\mesh_trimmer.ply -o tmp\\mesh_trimmer.ply -s meshlab\\script4color.mlx