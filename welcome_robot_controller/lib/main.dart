import 'dart:html';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:http/http.dart' as http;

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
  final String videoFeedUrl =
      'http://127.0.0.1:5000/video_feed?ts=${DateTime.now().millisecondsSinceEpoch}';

  @override
  void initState() {
    super.initState();

    // Register iframe for video stream
    ui.platformViewRegistry.registerViewFactory(
      'videoFeed',
      (int viewId) => IFrameElement()
        ..width = '100%'
        ..height = '100%'
        ..src = videoFeedUrl
        ..style.border = 'none',
    );
  }

  Future<void> sendDirection(String direction) async {
    const String apiUrl = 'http://127.0.0.1:5000/move';
    try {
      final response = await http.post(
        Uri.parse(apiUrl),
        headers: {'Content-Type': 'application/json'},
        body: '{"direction": "$direction"}',
      );
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
          // Video Stream
          Expanded(
            flex: 3,
            child: Container(
              margin: const EdgeInsets.all(10),
              decoration: BoxDecoration(border: Border.all(color: Colors.black)),
              child: const HtmlElementView(viewType: 'videoFeed'),
            ),
          ),

          // Joystick
          Expanded(
            flex: 2,
            child: Center(
              child: Joystick(
                mode: JoystickMode.all,
                listener: (details) {
                  final x = details.x;
                  final y = details.y;

                  // Decide direction
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
