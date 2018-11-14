#### Init submodules

`git submodule update --init --recursive`

#### Start the server

`./run_server.sh`

#### Open webapp

<http://localhost:8080>

#### Start shoveling pickles

`python ./shovel_pcls.py <num_points> <period_in_seconds>`

e.g.

`python ./shovel_pcls.py 5000 0.1  # 50,000 points/second`

#### Change the compression type

It is in [bench.js](webapp/js/bench.js)
