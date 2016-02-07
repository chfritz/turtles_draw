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

    var x = 200;
    (function animate() {

      var g = new PIXI.Graphics();
      g.moveTo(x, 200);
      g.lineStyle(4, 0xffd900, 1);
      x += 0.1;
      g.lineTo(x, 200);
      stage.addChild(g);

      renderer.render(stage);
      requestAnimationFrame( animate );
    })();
});
