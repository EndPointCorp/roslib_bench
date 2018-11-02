function init() {
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  var tfClient = new ROSLIB.TFClient({
    ros: ros,
  });

  var stats = new Stats();
  stats.showPanel(1);
  document.body.appendChild(stats.dom);

  var viewer = new ROS3D.Viewer({
    divID: 'viewer',
    antialias: true,
    width: window.innerWidth,
    height: window.innerHeight,
  });
  var wrappedDraw = viewer.draw.bind(viewer);
  var animateList = [];

  var started = false;
  var startedAt;
  var frames = 0;
  stats.begin();
  viewer.draw = function() {

    for (var i in animateList) {
      animateList[i]();
    }

    wrappedDraw();

    stats.end();

    if (started) {
      frames++;
    }
    if (frames === 100) {
      var report = document.createElement('p');
      report.style.position = 'fixed';
      report.style.top = '0px';
      report.style.right = '0px';
      report.style.color = 'red';
      document.body.appendChild(report);
      var dt = window.performance.now() - startedAt;
      report.innerText = '1000 frames in ' + dt;
      console.log('1000 frames in', dt);
    }

    stats.begin();
  };

  var t0 = 0;
  var camAngle = 0;
  function animateCamera() {
    var now = window.performance.now();
    var dt = now - t0;
    t0 = now;

    camAngle += dt * 0.001;

    viewer.camera.position.set(
      Math.sin(camAngle),
      Math.cos(camAngle),
      3
    );
    viewer.camera.lookAt(0, 0, 0);
  }
  animateList.push(animateCamera);

  viewer.addObject(new ROS3D.Grid());

  var pointLight = new THREE.PointLight(0xffffff, 0.3, 6.0);
  pointLight.castShadow = true;
  pointLight.position.set(0, 0, 0.9);
  viewer.scene.add(pointLight);

  var points = BenchPointCloud(ros, "/bench/pcl", "cbor");
  viewer.addObject(points.pointAnchor);
  animateList.push(points.animator);
  points.notify(function() {
    if (!started) {
      started = true;
      startedAt = window.performance.now();
    }
  });

  /*
  var map = new ROS3D.OccupancyGridClient({
    ros: ros,
    topic: "/einstein/map",
    rootObject: viewer.scene,
    continuous: true,
  });
  */
}

init();
