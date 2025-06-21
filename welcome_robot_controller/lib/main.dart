import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:http/http.dart' as http;
import 'package:flutter_mjpeg/flutter_mjpeg.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Welcome Robot Controller',
      home: RobotControllerPage(),
    );
  }
}

class RobotControllerPage extends StatefulWidget {
  const RobotControllerPage({super.key});

  @override
  State<RobotControllerPage> createState() => _RobotControllerPageState();
}

class _RobotControllerPageState extends State<RobotControllerPage> {
  // ⚠️ Replace with your PC's local IP
  final String serverIp = '192.168.1.3';

  late final String videoFeedUrl;
  late final String apiUrl;

  @override
  void initState() {
    super.initState();
    videoFeedUrl = 'http://$serverIp:5000/video_feed';
    apiUrl = 'http://$serverIp:5000/move';
  }

  Future<void> sendDirection(String direction) async {
    try {
      final response = await http
          .post(
            Uri.parse(apiUrl),
            headers: {'Content-Type': 'application/json'},
            body: '{"direction": "$direction"}',
          )
          .timeout(const Duration(seconds: 15)); // 🕒 Extended timeout
      print('Sent "$direction": ${response.statusCode}');
    } catch (e) {
      print('Error sending direction "$direction": $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Welcome Robot Controller'),
        centerTitle: true,
      ),
      body: Column(
        children: [
          // 📷 Video Stream
          Expanded(
            flex: 3,
            child: Container(
              margin: const EdgeInsets.all(10),
              decoration: BoxDecoration(border: Border.all(color: Colors.black)),
              child: Mjpeg(
                stream: videoFeedUrl,
                fit: BoxFit.cover,
                isLive: true,
                timeout: const Duration(seconds: 30), // 🛠️ Increase timeout
                error: (context, error, stack) => Text('Stream error: $error'),
              ),
            ),
          ),

          // 🎮 Joystick
          Expanded(
            flex: 2,
            child: Center(
              child: Joystick(
                mode: JoystickMode.all,
                listener: (details) {
                  final x = details.x;
                  final y = details.y;

                  if (y < -0.7) {
                    sendDirection('forward');
                  } else if (y > 0.7) {
                    sendDirection('backward');
                  } else if (x < -0.7) {
                    sendDirection('left');
                  } else if (x > 0.7) {
                    sendDirection('right');
                  } else {
                    sendDirection('stop');
                  }
                },
              ),
            ),
          ),
        ],
      ),
    );
  }
}
