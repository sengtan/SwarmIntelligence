import 'package:flutter/foundation.dart';
import 'package:web_socket_channel/io.dart';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:get_ip/get_ip.dart';
import 'dart:async';
import 'dart:io';

void main() => runApp(MyApp());
class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final title = 'WebSocket Demo';
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: title,
      home: MyHomePage(),
    );
  }
}

enum SocketState{
  Connected,
  notConnected
}
class MyHomePage extends StatefulWidget {

  @override
  _MyHomePageState createState() => _MyHomePageState();
}
class _MyHomePageState extends State<MyHomePage> {
  TextEditingController _controller = TextEditingController();
  SocketState socketState = SocketState.notConnected;
  WebSocketChannel channel;
  final String socketIP = "192.168.43.111";
  final String socketPort = "81";

  Future _getIP() async {
    String ipAddress = await GetIp.ipAddress;
    print(ipAddress);
  }
  void _connectWebSocket(){
    channel = IOWebSocketChannel.connect('ws://$socketIP:$socketPort/');
  }
  Widget _DisconnectButton(){
    return FlatButton(
      onPressed: (){
        channel.sink.close();
        setState(() {
          socketState = SocketState.notConnected;
        });
      },
      child: Text("Disconnect", style: TextStyle(color: Colors.white))
    );
  }
  Widget _ReconnectButton(){
    return FlatButton(
        onPressed: (){
          try{
            _connectWebSocket();
            setState(() {
            socketState = SocketState.Connected;
            });
          }
          on Exception catch (_) {
            channel.sink.close();
            setState(() {
              socketState = SocketState.notConnected;
            });
          }
        },
        child: Text("Reconnect",style: TextStyle(color: Colors.white))
    );
  }
  List<Widget> _appBar(){
    List<Widget> appbar = [];
    if(socketState == SocketState.Connected){
      appbar.add(_DisconnectButton());
      appbar.add(_ReconnectButton());
      return appbar;
    }
    else
      return [];
  }
  Widget _SocketButton(){
    switch(socketState){
      case SocketState.notConnected:
        return Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            RaisedButton(
              onPressed: () {
                _connectWebSocket();
                setState(() {
                  socketState = SocketState.Connected;
                });
              },
              child: Text("Connect", style: TextStyle(fontSize: 24.0),),
            ),
          ],
        );
        break;
      case SocketState.Connected:
        return Column(
          mainAxisAlignment: MainAxisAlignment.start,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            WebSocketDisplayer(channel)
          ],
        );
        break;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      resizeToAvoidBottomInset: false,
      appBar: AppBar(
        title: Text("Swarm WebSocket"),
        actions: _appBar()
      ),
      body: Padding(
        padding: const EdgeInsets.all(0),
        child: Container(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.start,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              _SocketButton()
            ],
          ),
        )
      ), // This trailing comma makes auto-formatting nicer for build methods.
    );
  }
}

class WebSocketDisplayer extends StatefulWidget{
  final WebSocketChannel channel;
  WebSocketDisplayer(this.channel);

  @override
  _WebSocketDisplayerState createState() => _WebSocketDisplayerState();
}
class _WebSocketDisplayerState extends State<WebSocketDisplayer>{
  TextEditingController _controller = TextEditingController();
  ScrollController _scrollController = ScrollController();
  final int Listlimit = 50;
  Map NodeData = {};

  @override
  void dispose() {
    widget.channel.sink.close();
    super.dispose();
  }
  void _sendMessage() {
    if (_controller.text.isNotEmpty) {
      widget.channel.sink.add(_controller.text);
      _controller.text = "";
    }
  }
  void _resetMessage(String mac) {
    if(NodeData.containsKey(mac)){
      NodeData[mac]["counter"] = 0;
      NodeData[mac]["history"].clear();
    }
  }

  // Widget processData(String rawdata){
  //   List<Widget> DisplayList = [];
  //   List datalist= rawdata.split(RegExp("\n"));
  //   datalist.removeLast();
  //   for(var data in datalist){
  //     /*
  //       retrieve the mac address and the data sent by that mac address
  //      */
  //     List specificdata = data.split(":");
  //     String mac = specificdata[0];
  //     String actualdata = specificdata[1];
  //     /*
  //       if first time collecting data for this mac address,
  //       instantiate counter and list
  //      */
  //     if(!NodeData.containsKey(mac)) {
  //       NodeData[mac] = {
  //         "counter":0,  //to keep track of how many data collected since
  //         "history":new List(),  //to keep the data collected
  //       };
  //     }
  //     /*
  //       if collected data is close to or more than Listlimit,
  //       remove until Listlimit-1 to make space for 1 incoming data
  //      */
  //     while(NodeData[mac]["history"].length>=(Listlimit-1)){
  //       NodeData[mac]["history"].removeAt(0);
  //     }
  //     NodeData[mac]["counter"]++; //increment to show how many data received alr
  //     NodeData[mac]["history"].add(actualdata); //add data to list of collected
  //     DisplayList.add(  //generate Listview widget tab and add to List to display
  //         Container(
  //           padding: EdgeInsets.fromLTRB(0.0,0.0,0.0,5.0),
  //           child: dataDisplayer(mac, NodeData[mac]["counter"], NodeData[mac]["history"]),
  //         )
  //     );
  //   }
  //   return Container(
  //     height: MediaQuery.of(context).size.height*0.84,//MediaQuery.of(context).size.height*0.8966,
  //     child: ListView(
  //       scrollDirection: Axis.vertical,
  //       physics:  AlwaysScrollableScrollPhysics(),
  //       padding: EdgeInsets.all(5.0),
  //       children: DisplayList,
  //     ),
  //   );
  // }
  Widget processData(String rawdata){
    List<Widget> DisplayList = [];
    List datalist= rawdata.split(RegExp("\n"));
    datalist.removeLast();
    /*
      retrieve the mac address and the data sent by that mac address
     */
    List specificdata = datalist[0].split(":");
    String mac = specificdata[0];
    String actualdata = specificdata[1];
    /*
      if first time collecting data for this mac address,
      instantiate counter and list
     */
    if(!NodeData.containsKey(mac)) {
      NodeData[mac] = {
        "counter":0,  //to keep track of how many data collected since
        "history":new List(),  //to keep the data collected
      };
    }
    /*
      if collected data is close to or more than Listlimit,
      remove until Listlimit-1 to make space for 1 incoming data
     */
    while(NodeData[mac]["history"].length>=(Listlimit-1)){
      NodeData[mac]["history"].removeAt(0);
    }
    NodeData[mac]["counter"]++; //increment to show how many data received alr
    NodeData[mac]["history"].add(actualdata); //add data to list of collected
    for (var key in NodeData.keys){
      DisplayList.add(  //generate Listview widget tab and add to List to display
          Container(
            padding: EdgeInsets.fromLTRB(0.0,0.0,0.0,5.0),
            child: dataDisplayer(key, NodeData[key]["counter"], NodeData[key]["history"]),
          )
      );
    }
    return Container(
      height: MediaQuery.of(context).size.height*0.84,//MediaQuery.of(context).size.height*0.8966,
      child: ListView(
        scrollDirection: Axis.vertical,
        physics:  AlwaysScrollableScrollPhysics(),
        padding: EdgeInsets.all(5.0),
        children: DisplayList,
      ),
    );
  }
  Widget dataDisplayer(String mac, int num, List data){
    List<Widget> textItems = [];
    int offset = data.length;
    int counter = 0;
    for(var item in data){
      counter = num-offset;
      offset--;
      textItems.add(
          Text("$counter:$item", textAlign: TextAlign.left,)
      );
    }
    return Container(
      color: Colors.green,
      padding: EdgeInsets.fromLTRB(10.0, 2.0, 10.0, 5.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Container(
                child: Text(mac, style: TextStyle(fontSize: 18.0, fontWeight: FontWeight.bold)),
              ),
              RaisedButton(
                onPressed: () => _resetMessage(mac),
                child: Text("Reset"),
              )
            ],
          ),
          Container(
            color: Colors.white,
            height:MediaQuery.of(context).size.height *0.12,
            child: Scrollbar(
              isAlwaysShown: true,
              controller: _scrollController,
              child:ListView(
                controller: _scrollController,
                shrinkWrap: true,
                scrollDirection: Axis.vertical,
                physics:  AlwaysScrollableScrollPhysics(),
                children: textItems,
              ),
            )
          )
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        StreamBuilder(
          stream: widget.channel.stream,
          builder: (context,snapshot){
            switch(snapshot.connectionState){
              case ConnectionState.waiting:
                return Text("Waiting for data...");
                break;
              case ConnectionState.active:
              case ConnectionState.done:
                if(snapshot.hasData)
                  return processData(snapshot.data);
                else
                  return Text("Done / No Data");
                break;
              default:
                return Text("Please disconnect");
            }
          },
        ),
        Container(
          height: MediaQuery.of(context).size.height*0.05,
          padding: EdgeInsets.fromLTRB(5.0, 0.0, 5.0, 0.0),
          child: Container(
              color: Colors.lightGreenAccent,
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceAround,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Container(
                    width: MediaQuery.of(context).size.width *0.70,
                    child: Form(
                      child: TextFormField(
                        controller: _controller,
                        decoration: InputDecoration(hintText: 'Send a message'),
                      ),
                    ),
                  ),
                  RaisedButton(
                    onPressed: _sendMessage,
                    child: Text("Send"),
                  )
                ],
              )
          )
        ),
      ],
    );
  }
}