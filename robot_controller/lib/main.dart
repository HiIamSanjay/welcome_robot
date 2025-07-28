import 'dart:async';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:http/http.dart' as http;
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'package:udp/udp.dart';

void main() {
  runApp(const RobotControllerApp());
}

class RobotControllerApp extends StatelessWidget {
  const RobotControllerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Robot Controller',
      theme: ThemeData.dark().copyWith(
        primaryColor: Colors.tealAccent,
        scaffoldBackgroundColor: const Color(0xFF121212),
        appBarTheme: const AppBarTheme(
          backgroundColor: Color(0xFF1F1F1F),
          elevation: 4,
        ),
      ),
      home: const DiscoveryPage(),
    );
  }
}

//--- Discovery Page: Finds the robot on the network ---
class DiscoveryPage extends StatefulWidget {
  const DiscoveryPage({super.key});

  @override
  State<DiscoveryPage> createState() => _DiscoveryPageState();
}

class _DiscoveryPageState extends State<DiscoveryPage> {
  String _status = "Searching for robot...";
  StreamSubscription<Datagram?>? _subscription;

  @override
  void initState() {
    super.initState();
    _startDiscovery();
  }

  Future<void> _startDiscovery() async {
    // Port must match the one in your Python server
    const int discoveryPort = 5000;
    
    try {
      final udp = await UDP.bind(Endpoint.any(port: Port(discoveryPort)));
      setState(() {
        _status = "Listening for robot broadcast on port $discoveryPort...";
      });
      
      // Listen for incoming packets
      _subscription = udp.asStream().listen((datagram) {
        if (datagram != null) {
          final String message = String.fromCharCodes(datagram.data);
          // Expected message format: "robot_server:<ip>"
          if (message.startsWith('robot_server:')) {
            final String serverIp = message.split(':')[1];
            print("🤖 Robot found at $serverIp!");
            _subscription?.cancel(); // Stop listening
            udp.close();
            
            // Navigate to the controller page
            Navigator.of(context).pushReplacement(
              MaterialPageRoute(
                builder: (context) => ControllerPage(serverIp: serverIp),
              ),
            );
          }
        }
      });
      
      // Timeout if robot is not found
      Future.delayed(const Duration(seconds: 20), () {
        if (mounted) {
           _subscription?.cancel();
           udp.close();
           setState(() {
             _status = "Could not find robot. Please ensure it's on the same Wi-Fi network and the server is running.";
           });
        }
      });

    } catch (e) {
      print("Error starting discovery: $e");
      setState(() {
        _status = "Error: Could not bind to discovery port. Is another app using it?";
      });
    }
  }

  @override
  void dispose() {
    _subscription?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Connecting...')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const CircularProgressIndicator(valueColor: AlwaysStoppedAnimation<Color>(Colors.tealAccent)),
            const SizedBox(height: 24),
            Text(
              _status,
              textAlign: TextAlign.center,
              style: const TextStyle(fontSize: 16, color: Colors.white70),
            ),
            const SizedBox(height: 20),
            ElevatedButton(
                onPressed: () {
                    setState(() { _status = "Retrying..."; });
                    _startDiscovery();
                },
                child: const Text("Retry"),
            ),
          ],
        ),
      ),
    );
  }
}

//--- Controller Page: The main UI for video and joystick ---
class ControllerPage extends StatefulWidget {
  final String serverIp;
  const ControllerPage({super.key, required this.serverIp});

  @override
  State<ControllerPage> createState() => _ControllerPageState();
}

class _ControllerPageState extends State<ControllerPage> {
  late final String videoFeedUrl;
  late final String apiUrl;
  String _lastDirection = 'stop';
  Timer? _stopTimer;

  @override
  void initState() {
    super.initState();
    videoFeedUrl = 'http://${widget.serverIp}:5000/video_feed';
    apiUrl = 'http://${widget.serverIp}:5000/move';
    print("Connecting to stream: $videoFeedUrl");
  }
  
  @override
  void dispose() {
    _stopTimer?.cancel();
    // Send a final stop command when leaving the page
    sendDirection('stop');
    super.dispose();
  }

  Future<void> sendDirection(String direction) async {
    // Avoid sending redundant commands
    if (_lastDirection == direction) return;
    _lastDirection = direction;

    // Debounce the stop command
    _stopTimer?.cancel();
    if (direction == 'stop') {
        _stopTimer = Timer(const Duration(milliseconds: 200), () {
            _executeSend(direction);
        });
    } else {
        _executeSend(direction);
    }
  }

  Future<void> _executeSend(String direction) async {
     try {
      await http.post(
        Uri.parse(apiUrl),
        headers: {'Content-Type': 'application/json'},
        body: '{"direction": "$direction"}',
      ).timeout(const Duration(seconds: 1));
      print('Sent "$direction"');
    } catch (e) {
      print('Error sending direction "$direction": $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Robot Controller | ${widget.serverIp}'),
        centerTitle: true,
        leading: const Icon(Icons.smart_toy_outlined),
      ),
      body: Stack(
        children: [
          // Video Feed (Background)
          Mjpeg(
            stream: videoFeedUrl,
            fit: BoxFit.cover,
            isLive: true,
            timeout: const Duration(seconds: 10),
            error: (context, error, stack) {
              return Center(
                child: Text('Stream Error: $error', style: const TextStyle(color: Colors.red)),
              );
            },
          ),
          
          // Joystick (Foreground)
          Align(
            alignment: Alignment.bottomCenter,
            child: Padding(
              padding: const EdgeInsets.all(30.0),
              child: Container(
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.3),
                  shape: BoxShape.circle,
                ),
                child: Joystick(
                  mode: JoystickMode.all,
                  listener: (details) {
                    final x = details.x;
                    final y = details.y;
                    const threshold = 0.5;

                    if (y < -threshold) sendDirection('forward');
                    else if (y > threshold) sendDirection('backward');
                    else if (x < -threshold) sendDirection('left');
                    else if (x > threshold) sendDirection('right');
                    else sendDirection('stop');
                  },
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}