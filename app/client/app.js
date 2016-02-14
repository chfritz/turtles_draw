
class Drawing {
  /** class to handling all the application logic of drawing traces of the
  turtle as well as capturing clicks sequences and sending them as shapes */

  constructor() {
    // setup pixi
    this.size = Math.min(window.innerWidth, window.innerHeight);
    this.renderer = PIXI.autoDetectRenderer(
      this.size, this.size, { antialias: true, autoResize: true });
    $("#canvas").get(0).appendChild(this.renderer.view);

    this.stage = new PIXI.Container();
    this.stage.interactive = true;
    this.renderer.view.onclick = this.onClick.bind(this);

    this.shape = [];
    this.last = null;

    // width and height of turtlesim coordinate system
    this.turtlesim_size = 11;

    // keep getting traces and draw them on updates
    Trace.find().observe({
        changed: this.update.bind(this),
        added: this.update.bind(this)
      });

    this.animate();
  }

  /** handling mouse clicks on the canvas: add nav point, check for closes
  /** shape, when closed, order turtle to draw */
  onClick(mouseData) {

    var coords = {
      // normalize coordinates in terms of window size (after resizing)
      x: mouseData.x * (this.size / this.renderer.view.clientWidth),
      y: mouseData.y * (this.size / this.renderer.view.clientHeight)
    }
    var turtle_coords = this.translate_inv(coords);
    console.log(turtle_coords);

    var distance;
    if (this.shape.length > 1) {

      diff = {
        x: turtle_coords.x - this.shape[0].x,
        y: turtle_coords.y - this.shape[0].y
      };
      distance = Math.sqrt(diff.x * diff.x + diff.y * diff.y);
    }

    if (distance && distance < 0.1) {
      // the user clicked very close to the origin, close this shape
      this.shape.push(this.shape[0]);
      // teleport turtle to the start
      Meteor.call('teleport', this.shape[0], (function(err, res) {
          if (err) {
            console.log("not drawing, couldn't teleport: ", err);
          } else {
            // clear the canvas
            this.last = null;
            this.stage.removeChildren();
            // then order the turtle to draw this
            Meteor.call('draw', this.shape);
            this.shape = [];
          }
      }).bind(this));

    } else {

      this.shape.push(turtle_coords);
      var g = new PIXI.Graphics();
      g.lineStyle(0);
      g.beginFill(0xFFFF0B, 0.5);
      g.drawCircle(coords.x, coords.y, 10);
      g.endFill();
      this.stage.addChild(g);
    }
  }

  // handling pose updates from server (via collection): draw consecutive polygon
  update(pose) {
    console.log("new pose: ", pose);

    var newpose = this.translate(pose);

    if (this.last) {
      var g = new PIXI.Graphics();
      g.lineStyle(4, 0xffd900, 1);
      g.moveTo(this.last.x, this.last.y);
      g.lineTo(newpose.x, newpose.y);
      this.stage.addChild(g);
    }

    this.last = newpose;
  }

  // animation loop
  animate() {
    this.renderer.render(this.stage);
    requestAnimationFrame(this.animate.bind(this));
  };

  // -------------------------------------------------
  // utilities

  /** translate from turtlesim coordinates to pixi coordinates */
  translate(pose) {
    var scale = this.size / this.turtlesim_size;
    return {
      x: pose.x * scale,
      y: this.size - (pose.y * scale)
    };
  }

  /** inverse of translate */
  translate_inv(coords) {
    console.log(coords);
    var scale = this.size / this.turtlesim_size;
    return {
      x: coords.x / scale,
      y: (this.size - coords.y) / scale
    };
  }

}

// -------------------------------------------------------------------
// Meteor code

Template.canvas.onCreated( function() {
    Meteor.subscribe('trace');
});

Template.canvas.onRendered( function() {
    var drawing = new Drawing();
});

// -------------------------------------------------------------------

// resize the canvas with the window
window.onresize = function() {
  var newsize = Math.min(window.innerWidth, window.innerHeight);
  $('canvas').get(0).style.width = newsize + "px";
  $('canvas').get(0).style.height = newsize + "px";
}
