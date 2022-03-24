import 'package:dio/dio.dart';
import 'package:flutter/material.dart';

String ip = "http://192.168.10.119:8080/";

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  MyHomePage({Key? key, required this.title}) : super(key: key);

  final String title;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  String connStat = "";

  Future<Response?> getHttp(String path) async {
    try {
      var response = await Dio(BaseOptions(connectTimeout: 5000)).get(path);
      return response;
    } catch (e) {
      print(e);
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(e.toString()),
        duration: Duration(seconds: 1),
      ));
      return null;
    }
  }

  void _incrementCounter() {
    getHttp(ip);
    setState(() {});
  }

  void ping() {
    getHttp(ip).then((value) {
      if (value != null) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
          content: Text("Recieved data!"),
          duration: Duration(seconds: 1),
        ));
        connStat = "Active";
        setState(() {});
      } else {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
            content: Text("Unsuccessful"), duration: Duration(seconds: 1)));
        connStat = "Disconnected";
        setState(() {});
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.start,
          children: <Widget>[
            SizedBox(
              height: MediaQuery.of(context).size.height / 10,
            ),
            InkWell(
              onTap: ping,
              child: Container(
                height: (MediaQuery.of(context).size.height / 10 < 100)
                    ? 100
                    : MediaQuery.of(context).size.height / 10,
                width: MediaQuery.of(context).size.width / 1.5,
                child: Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Card(
                    child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Text(
                            "Ping",
                            style: TextStyle(
                                fontSize: 20, fontWeight: FontWeight.bold),
                          ),
                          Text("IP: $ip"),
                          Text(connStat)
                        ]),
                    elevation: 5,
                  ),
                ),
              ),
            ),
            IconButton(onPressed: null, icon: Icon(Icons.arrow_upward)),
            Row( mainAxisAlignment: MainAxisAlignment.center,
              children: [
              IconButton(onPressed: null, icon: Icon(Icons.arrow_left)),
              SizedBox(width: MediaQuery.of(context).size.width / 3),
                 IconButton(onPressed: null, icon: Icon(Icons.arrow_right)),
            ],),
            IconButton(onPressed: null, icon: Icon(Icons.arrow_downward)),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _incrementCounter,
        tooltip: 'Increment',
        child: Icon(Icons.add),
      ),
      // This trailing comma makes auto-formatting nicer for build methods.
    );
  }
}
