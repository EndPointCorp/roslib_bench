function BenchPointCloud(ros, topic, compression) {
  var pointBuffer = null;
  var pointNode = null;
  var pointRatio = 1;  // for subsampling
  var maxPoints = 5000;
  var pointMeshes = [];
  var pointAnchor = new THREE.Object3D();
  var boxGeometry = new THREE.BoxBufferGeometry(0.01, 0.01, 0.01);
  var lastMessage = null;
  var dirty = false;
  var notifier = null;

  for (var i = 0; i < maxPoints; i++) {
    var box = new THREE.Mesh(
      boxGeometry,
      new THREE.MeshPhongMaterial({ color: 0xffffff })
    );

    box.visible = false;
    box.material.emissiveIntensity = 0.25;
    box.material.shininess = 10;
    pointAnchor.add(box);
    pointMeshes.push(box);
  }

  var pclTopic = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: 'sensor_msgs/PointCloud2',
    compression: compression,
    throttle_rate: 16,
    queue_length: 1
  });

  pclTopic.subscribe((msg) => {
    lastMessage = msg;
    dirty = true;
    if (notifier) {
      notifier();
    }
  });

  function animatePoints() {
    if (!dirty) {
      return;
    }
    dirty = false;

    var msg = lastMessage;

    // lazy init the buffer
    if (!pointBuffer) {
      pointBuffer = new Uint8Array(maxPoints * msg.point_step);
    }

    var numPoints;

    if (msg.data.buffer) {
      pointBuffer.set(msg.data.subarray(0, maxPoints * msg.point_step));
      //pointBuffer = msg.data;
      numPoints = msg.height*msg.width / pointRatio;
      //pointBuffer = msg.data;
    } else {
      numPoints = decode64(msg.data, pointBuffer, msg.point_step, pointRatio);
    }

    var dv = new DataView(pointBuffer.buffer);
    var littleEndian = !msg.is_bigendian;

    // turn fields to a map
    var fields = {};
    for (var i = 0; i < msg.fields.length; i++) {
      fields[msg.fields[i].name] = msg.fields[i];
    }

    var x = fields.x.offset;
    var y = fields.y.offset;
    var z = fields.z.offset;
    var intensity = fields.intensity.offset;

    for (var i = 0; i < maxPoints; i++) {
      var mesh = pointMeshes[i];

      if (i >= numPoints) {
        mesh.visible = false;
        continue;
      }

      mesh.visible = true;
      var base = i * pointRatio * msg.point_step;
      mesh.position.set(
        dv.getFloat32(base+x, littleEndian),
        dv.getFloat32(base+y, littleEndian),
        dv.getFloat32(base+z, littleEndian)
      );

      var purple = [130/255, 86/255, 232/255];
      var pink = [243/255, 95/255, 151/255];
      var orange = [232/255, 158/255, 0];
      var minDist = 0.35;
      var maxDist = 1.0;
      var distance = Math.max(0, Math.min(maxDist, mesh.position.length() * 0.6 - minDist));
      var midPoint = 0.66;
      var r, g, b;
      if (distance < midPoint) {
        r = purple[0] + distance / midPoint * (pink[0] - purple[0]);
        g = purple[1] + distance / midPoint * (pink[1] - purple[1]);
        b = purple[2] + distance / midPoint * (pink[2] - purple[2]);
      } else {
        r = pink[0] + (distance - midPoint) / (1.0 - midPoint) * (orange[0] - pink[0]);
        g = pink[1] + (distance - midPoint) / (1.0 - midPoint) * (orange[1] - pink[1]);
        b = pink[2] + (distance - midPoint) / (1.0 - midPoint) * (orange[2] - pink[2]);
      }
      mesh.material.color.setRGB(r, g, b);
      mesh.material.emissive.setRGB(r, g, b);
      mesh.material.specular.setRGB(r, g, b);

      var scale = dv.getFloat32(base+intensity, littleEndian) * 0.075;
      mesh.scale.set(scale, scale, scale);
    }
  }

  return {
    pointAnchor: pointAnchor,
    animator: animatePoints,
    notify: function(cb) {
      notifier = cb;
    },
  };
}
