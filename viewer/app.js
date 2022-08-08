let express = require("express");
let http_server = require("http").Server(express());
let io_server = require("socket.io")(http_server, {
  maxHttpBufferSize: 1e12,
  pingTimeout: 120000
});

let app = express();
let http_publisher = require("http").Server(app);
let io_publisher = require("socket.io")(http_publisher, {
  maxHttpBufferSize: 1e12,
  pingTimeout: 120000
});

// Port variables
let PORT_SERVER = 3000
let PORT_PUBLISHER = 3001
if (process.argv.length > 2) {
PORT_SERVER = process.argv[2]
}
if (process.argv.length > 3) {
PORT_PUBLISHER = process.argv[3]
}

// setting express
app.set("views", __dirname + "/views");
app.set("view engine", "ejs");
app.use(express.static(__dirname + "/public"));

// render browser
app.get("/", function (req, res) {
  res.render("index.ejs");
});

io_server.on("connection", function (socket) {
  console.log(`Connected - ID: ${socket.id}`);

  socket.on("map_publish", function (msg) {
    io_publisher.emit("map_publish", msg);
  });

  socket.on("frame_publish", function (msg) {
    io_publisher.emit("frame_publish", { image: true, buffer: msg });
  });

  socket.on("disconnect", function () {
    console.log(`Disconnected - ID: ${socket.id}`);
  });
});

io_publisher.on("connection", function (socket) {
  socket.on("signal", function (msg) {
    io_server.emit("signal", msg);
  });
});

http_server.listen(PORT_SERVER, function () {
  console.log("WebSocket: listening on *:",PORT_SERVER);
});

http_publisher.listen(PORT_PUBLISHER, function () {
  console.log("HTTP server: listening on *:",PORT_PUBLISHER)
});
