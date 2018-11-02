#### Init submodules

git submodule update --init --recursive

#### Start the server

`./run_server.sh`

#### Open webapp

<http://localhost:8080>

#### Start shoveling pickles

`python ./shovel_pcls.py <num_points> <period_in_seconds>`

e.g.

`python ./shovel_pcls.py 10000 0.1  # 100,000 points/second`
