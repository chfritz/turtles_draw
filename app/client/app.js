Session.setDefault('counter', 0);

Template.canvas.helpers({
  });

Template.canvas.events({
  });

Template.canvas.onRendered( function(){

    var renderer = PIXI.autoDetectRenderer(800, 600, { antialias: true });
    $("#canvas").get(0).appendChild(renderer.view);

    // create the root of the scene graph
    var stage = new PIXI.Container();
    stage.interactive = true;

    function animate() {
      renderer.render(stage);
      requestAnimationFrame( animate );
    };

    Meteor.subscribe('trace');

    var last = null;

    function transpose(pose) {
      var SCALE = 50;
      var OFFSET = { x: 100, y: 100 };
      pose.x = pose.x * SCALE + OFFSET.x;
      pose.y = pose.y * SCALE + OFFSET.y;
    }

    function update(id, fields) {
      transpose(fields);
      console.log("changed", fields);
      if (last) {
        var g = new PIXI.Graphics();
        g.lineStyle(4, 0xffd900, 1);
        g.moveTo(last.x, last.y);
        g.lineTo(fields.x, fields.y);
        stage.addChild(g);
      }

      last = fields;
    }

    Trace.find().observeChanges({
        changed: update,
        added: update
      });

    // start animation
    animate();
});
