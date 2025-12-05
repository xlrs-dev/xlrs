import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter/services.dart';
import 'package:permission_handler/permission_handler.dart';
import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';

// BLE Service and Characteristic UUIDs (must match RX implementation)
final Guid fpvServiceUUID = Guid("0000ff00-0000-1000-8000-00805f9b34fb");
final Guid txCharUUID = Guid("0000ff01-0000-1000-8000-00805f9b34fb");
final Guid rxCharUUID = Guid("0000ff02-0000-1000-8000-00805f9b34fb");

void main() {
  runApp(const FPVRXConfigApp());
}

class FPVRXConfigApp extends StatelessWidget {
  const FPVRXConfigApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'FPV RX Config',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        useMaterial3: true,
      ),
      home: const MainScreen(),
    );
  }
}

class MainScreen extends StatefulWidget {
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> {
  List<ScanResult> foundDevices = [];
  bool isScanning = false;
  BluetoothDevice? connectedDevice;
  BluetoothCharacteristic? txCharacteristic;
  BluetoothCharacteristic? rxCharacteristic;
  String deviceName = '';
  String currentPIN = '';
  String? errorMessage;
  bool isLoading = false;
  StreamSubscription<List<int>>? _notificationSubscription;
  StringBuffer _responseBuffer = StringBuffer();

  @override
  void initState() {
    super.initState();
    _requestPermissions();
    _setupBluetoothListeners();
  }

  void _setupBluetoothListeners() {
    // Listen for adapter state changes
    FlutterBluePlus.adapterState.listen((state) {
      if (state == BluetoothAdapterState.off) {
        setState(() {
          errorMessage = "Bluetooth is turned off. Please enable Bluetooth.";
        });
      }
    });
  }

  Future<void> _requestPermissions() async {
    try {
      // Request location permission (required for Bluetooth scanning on Android)
      PermissionStatus locationStatus = await Permission.location.request();
      
      if (locationStatus.isDenied || locationStatus.isPermanentlyDenied) {
        setState(() {
          errorMessage = "Location permission is required for Bluetooth scanning.\n"
              "Please grant location permission in Settings.";
        });
        return;
      }
      
      // Request Bluetooth permissions
      if (await FlutterBluePlus.isSupported == false) {
        setState(() {
          errorMessage = "Bluetooth is not supported on this device";
        });
        return;
      }
      
      // Turn on Bluetooth if needed
      if (await FlutterBluePlus.adapterState.first == BluetoothAdapterState.off) {
        await FlutterBluePlus.turnOn();
      }
    } catch (e) {
      setState(() {
        errorMessage = "Permission error: $e\nPlease grant Bluetooth and Location permissions in Settings";
      });
    }
  }

  Future<void> _scanForDevices() async {
    setState(() {
      isScanning = true;
      foundDevices.clear();
      errorMessage = null;
    });

    try {
      // Check and request location permission first
      PermissionStatus locationStatus = await Permission.location.status;
      if (!locationStatus.isGranted) {
        locationStatus = await Permission.location.request();
        if (!locationStatus.isGranted) {
          setState(() {
            errorMessage = "Location permission is required for Bluetooth scanning.\n"
                "Please grant location permission when prompted.";
            isScanning = false;
          });
          return;
        }
      }
      
      // Check Bluetooth adapter state
      if (await FlutterBluePlus.adapterState.first != BluetoothAdapterState.on) {
        setState(() {
          errorMessage = "Please enable Bluetooth in Settings";
          isScanning = false;
        });
        return;
      }
      
      // Start scanning for BLE devices (scan all, filter in listener)
      // Note: Don't use withServices filter as it might be too restrictive
      // Some devices may not advertise service UUIDs in scan response
      FlutterBluePlus.startScan(timeout: const Duration(seconds: 15));
      
      FlutterBluePlus.scanResults.listen((results) {
        for (ScanResult result in results) {
          // Get device name
          String name = result.device.platformName.isNotEmpty 
              ? result.device.platformName 
              : "";
          
          // Check if device advertises our FPV service UUID
          bool hasFPVService = result.advertisementData.serviceUuids.contains(fpvServiceUUID);
          
          // Also check complete local name in advertisement data
          String localName = result.advertisementData.localName;
          
          // Match by name patterns (since service UUIDs aren't being advertised)
          // Accept devices with "BTstack" in name (likely our RX) or "FPV_RX_" prefix
          bool matchesName = name.toLowerCase().contains("btstack") || 
                           name.startsWith("FPV_RX_") || 
                           localName.toLowerCase().contains("btstack") ||
                           localName.startsWith("FPV_RX_");
          
          // Debug: print all devices for troubleshooting (only first time we see each device)
          if (!foundDevices.any((d) => d.device.remoteId == result.device.remoteId)) {
            print("Scan result: ${result.device.remoteId}, name: '$name', localName: '$localName', "
                "serviceUuids: ${result.advertisementData.serviceUuids}, hasFPVService: $hasFPVService, matchesName: $matchesName");
          }
          
          // Accept devices that advertise our service OR match name pattern
          // For now, accept BTstack devices since service UUIDs aren't being advertised
          if (hasFPVService || matchesName) {
            // Check if we already have this device
            if (!foundDevices.any((d) => d.device.remoteId == result.device.remoteId)) {
              setState(() {
                foundDevices.add(result);
                print("✓ Added FPV RX device: ${result.device.remoteId}, name: '$name'");
              });
            }
          }
        }
      });
      
      // Wait for scan to complete
      await Future.delayed(const Duration(seconds: 15));
      await FlutterBluePlus.stopScan();
      
      setState(() {
        isScanning = false;
      });
      
      if (foundDevices.isEmpty) {
        setState(() {
          errorMessage = "No FPV RX devices found. Make sure:\n"
              "1. RX device is powered on\n"
              "2. BLE is enabled and advertising\n"
              "3. Device advertises FPV service UUID\n"
              "4. Location permission is granted\n"
              "\nNote: Scanning filters by service UUID, not device name.";
        });
      }
    } catch (e) {
      setState(() {
        errorMessage = "Scan error: $e";
        isScanning = false;
      });
    }
  }

  Future<void> _connectToDevice(ScanResult scanResult) async {
    setState(() {
      isLoading = true;
      errorMessage = null;
    });

    try {
      BluetoothDevice device = scanResult.device;
      
      // Connect to the device
      await device.connect(timeout: const Duration(seconds: 15), autoConnect: false);
      
      // Discover services
      List<BluetoothService> services = await device.discoverServices();
      
      // Find our FPV service
      BluetoothService? fpvService;
      for (BluetoothService service in services) {
        if (service.serviceUuid == fpvServiceUUID) {
          fpvService = service;
          break;
        }
      }
      
      if (fpvService == null) {
        throw Exception("FPV service not found on device");
      }
      
      // Find TX and RX characteristics
      BluetoothCharacteristic? txChar;
      BluetoothCharacteristic? rxChar;
      
      for (BluetoothCharacteristic char in fpvService.characteristics) {
        if (char.characteristicUuid == txCharUUID) {
          txChar = char;
        } else if (char.characteristicUuid == rxCharUUID) {
          rxChar = char;
        }
      }
      
      if (txChar == null) {
        throw Exception("TX characteristic not found");
      }
      
      setState(() {
        connectedDevice = device;
        txCharacteristic = txChar;
        rxCharacteristic = rxChar;
        deviceName = device.platformName.isNotEmpty ? device.platformName : device.remoteId.toString();
      });
      
      // Subscribe to RX characteristic notifications if available
      if (rxChar != null && rxChar.properties.notify) {
        await rxChar.setNotifyValue(true);
        _notificationSubscription = rxChar.onValueReceived.listen((value) {
          // Handle notifications if needed
          String received = utf8.decode(value);
          _responseBuffer.write(received);
          _handleResponse(_responseBuffer.toString());
        });
      }
      
      // Read current PIN (if we have a way to do this via BLE)
      // For now, we'll need to add a control characteristic for configuration
      // For this version, we'll just show the UI
      
    } catch (e) {
      setState(() {
        errorMessage = "Connection failed: $e";
        connectedDevice = null;
        txCharacteristic = null;
        rxCharacteristic = null;
      });
      if (connectedDevice != null) {
        await connectedDevice!.disconnect();
      }
    } finally {
      setState(() {
        isLoading = false;
      });
    }
  }

  Future<void> _readCurrentPIN() async {
    // Note: This requires a control characteristic for configuration
    // For now, this is a placeholder
    // We'll need to add a control characteristic to the RX side
    if (txCharacteristic == null) return;
    
    try {
      // Send GETPIN command via TX characteristic
      // This is a workaround - ideally we'd have a separate control characteristic
      Uint8List command = Uint8List.fromList(utf8.encode("GETPIN\n"));
      await txCharacteristic!.write(command, withoutResponse: false);
      
      // Wait for response
      await Future.delayed(const Duration(milliseconds: 500));
    } catch (e) {
      setState(() {
        errorMessage = "Failed to read PIN: $e";
      });
    }
  }

  Future<void> _setPIN(String pin) async {
    if (txCharacteristic == null) return;
    
    // Validate PIN format (6 numeric digits for BLE)
    if (pin.length != 6 || !RegExp(r'^[0-9]{6}$').hasMatch(pin)) {
      setState(() {
        errorMessage = "Invalid PIN format. Must be 6 numeric digits (0-9)";
      });
      return;
    }
    
    setState(() {
      isLoading = true;
      errorMessage = null;
    });

    try {
      // Send SETPASSKEY command via TX characteristic
      // Note: This is a workaround - ideally we'd have a separate control characteristic
      String command = "SETPASSKEY:${pin}\n";
      Uint8List data = Uint8List.fromList(utf8.encode(command));
      await txCharacteristic!.write(data, withoutResponse: false);
      
      // Wait for response
      await Future.delayed(const Duration(milliseconds: 500));
      
      // Read PIN again to confirm
      await _readCurrentPIN();
      
      setState(() {
        currentPIN = pin;
      });
      
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text("PIN updated successfully!")),
        );
      }
    } catch (e) {
      setState(() {
        errorMessage = "Failed to set PIN: $e";
      });
    } finally {
      setState(() {
        isLoading = false;
      });
    }
  }

  void _handleResponse(String response) {
    // Parse responses from RX device
    if (response.contains("PASSKEY:")) {
      // Extract PIN from "PASSKEY: XXXXXX" format
      RegExp pinRegex = RegExp(r'PASSKEY:\s*([0-9]{6})');
      Match? match = pinRegex.firstMatch(response);
      if (match != null) {
        setState(() {
          currentPIN = match.group(1)!;
        });
        _responseBuffer.clear();
      }
    } else if (response.contains("OK")) {
      // Command succeeded
      setState(() {
        errorMessage = null;
      });
      _responseBuffer.clear();
    } else if (response.contains("ERROR")) {
      // Command failed
      setState(() {
        errorMessage = response.trim();
      });
      _responseBuffer.clear();
    }
  }

  Future<void> _disconnect() async {
    _notificationSubscription?.cancel();
    _notificationSubscription = null;
    
    if (rxCharacteristic != null) {
      try {
        await rxCharacteristic!.setNotifyValue(false);
      } catch (e) {
        // Ignore errors when disconnecting
      }
    }
    
    if (connectedDevice != null) {
      try {
        await connectedDevice!.disconnect();
      } catch (e) {
        // Ignore errors when disconnecting
      }
    }
    
    setState(() {
      connectedDevice = null;
      txCharacteristic = null;
      rxCharacteristic = null;
      deviceName = '';
      currentPIN = '';
    });
  }

  void _showPINEntryDialog() {
    final TextEditingController pinController = TextEditingController(
      text: currentPIN,
    );
    
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Set Passkey'),
        content: TextField(
          controller: pinController,
          maxLength: 6,
          keyboardType: TextInputType.number,
          decoration: const InputDecoration(
            labelText: '6-digit numeric passkey (0-9)',
            hintText: '123456',
          ),
          inputFormatters: [
            FilteringTextInputFormatter.digitsOnly,
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
          TextButton(
            onPressed: () {
              String pin = pinController.text;
              if (pin.length == 6) {
                _setPIN(pin);
                Navigator.pop(context);
              } else {
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(
                    content: Text("Passkey must be exactly 6 numeric digits"),
                  ),
                );
              }
            },
            child: const Text('Set'),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('FPV RX Config (BLE)'),
        actions: [
          if (connectedDevice != null)
            IconButton(
              icon: const Icon(Icons.close),
              onPressed: _disconnect,
              tooltip: 'Disconnect',
            ),
        ],
      ),
      body: RefreshIndicator(
        onRefresh: () async {
          if (!isScanning) {
            await _scanForDevices();
          }
        },
        child: SingleChildScrollView(
          physics: const AlwaysScrollableScrollPhysics(),
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              // Error message
              if (errorMessage != null)
                Container(
                  padding: const EdgeInsets.all(12),
                  margin: const EdgeInsets.only(bottom: 16),
                  decoration: BoxDecoration(
                    color: Colors.red.shade100,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.red),
                  ),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          const Icon(Icons.error, color: Colors.red),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              errorMessage!,
                              style: const TextStyle(color: Colors.red),
                            ),
                          ),
                        ],
                      ),
                      if (errorMessage!.toLowerCase().contains('permission') ||
                          errorMessage!.toLowerCase().contains('location'))
                        Padding(
                          padding: const EdgeInsets.only(top: 8.0),
                          child: Text(
                            'Go to Settings > Apps > FPV RX Config > Permissions\n'
                            'and grant Location permission.',
                            style: TextStyle(
                              color: Colors.red.shade700,
                              fontSize: 12,
                            ),
                          ),
                        ),
                    ],
                  ),
                ),
              
              // Connected device info
              if (connectedDevice != null) ...[
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(16.0),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Row(
                          children: [
                            const Icon(Icons.bluetooth_connected, color: Colors.green),
                            const SizedBox(width: 8),
                            Expanded(
                              child: Text(
                                deviceName,
                                style: const TextStyle(
                                  fontSize: 18,
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 8),
                        Text(
                          'MAC: ${connectedDevice!.remoteId}',
                          style: const TextStyle(
                            fontFamily: 'monospace',
                            fontSize: 12,
                            color: Colors.grey,
                          ),
                        ),
                        const SizedBox(height: 16),
                        const Text(
                          'Current Passkey:',
                          style: TextStyle(fontSize: 14, color: Colors.grey),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          currentPIN.isEmpty ? 'Not available' : currentPIN,
                          style: const TextStyle(
                            fontSize: 24,
                            fontWeight: FontWeight.bold,
                            letterSpacing: 2,
                          ),
                        ),
                        const SizedBox(height: 16),
                        SizedBox(
                          width: double.infinity,
                          child: ElevatedButton.icon(
                            onPressed: isLoading ? null : _showPINEntryDialog,
                            icon: const Icon(Icons.edit),
                            label: const Text('Set Passkey'),
                          ),
                        ),
                        const SizedBox(height: 8),
                        Text(
                          'Note: Passkey configuration requires a control characteristic.\n'
                          'This feature may need to be added to the RX firmware.',
                          style: TextStyle(
                            fontSize: 12,
                            color: Colors.grey.shade600,
                            fontStyle: FontStyle.italic,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 16),
              ],
              
              // Scan button
              if (connectedDevice == null) ...[
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton.icon(
                    onPressed: isScanning ? null : _scanForDevices,
                    icon: isScanning
                        ? const SizedBox(
                            width: 20,
                            height: 20,
                            child: CircularProgressIndicator(strokeWidth: 2),
                          )
                        : const Icon(Icons.search),
                    label: Text(isScanning ? 'Scanning...' : 'Scan for Devices'),
                  ),
                ),
                const SizedBox(height: 16),
              ],
              
              // Device list
              if (foundDevices.isEmpty && !isScanning && connectedDevice == null)
                Padding(
                  padding: const EdgeInsets.all(32.0),
                  child: Center(
                    child: Column(
                      children: [
                        const Text(
                          'No FPV RX devices found.\nTap "Scan for Devices" to search.',
                          textAlign: TextAlign.center,
                          style: TextStyle(color: Colors.grey),
                        ),
                        const SizedBox(height: 16),
                        const Text(
                          'Note: Make sure your RX device is:\n'
                          '• Powered on\n'
                          '• BLE advertising\n'
                          '• Named "FPV_RX_XXX"',
                          textAlign: TextAlign.center,
                          style: TextStyle(color: Colors.grey, fontSize: 12),
                        ),
                      ],
                    ),
                  ),
                )
              else if (foundDevices.isNotEmpty && connectedDevice == null)
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Padding(
                      padding: const EdgeInsets.symmetric(vertical: 8.0),
                      child: Text(
                        'Found ${foundDevices.length} device(s):',
                        style: Theme.of(context).textTheme.titleSmall,
                      ),
                    ),
                    ...foundDevices.map((scanResult) {
                      final device = scanResult.device;
                      final name = device.platformName.isNotEmpty 
                          ? device.platformName 
                          : "Unknown Device";
                      final address = device.remoteId.toString();
                      final rssi = scanResult.rssi;
                      final isFPVRX = name.startsWith("FPV_RX_");
                      
                      return Card(
                        margin: const EdgeInsets.only(bottom: 8),
                        child: ListTile(
                          leading: Icon(
                            Icons.bluetooth,
                            color: isFPVRX ? Colors.blue : Colors.grey,
                          ),
                          title: Text(name),
                          subtitle: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                'MAC: $address',
                                style: const TextStyle(fontFamily: 'monospace', fontSize: 12),
                              ),
                              Text(
                                'RSSI: $rssi dBm',
                                style: const TextStyle(fontSize: 12, color: Colors.grey),
                              ),
                            ],
                          ),
                          trailing: isLoading
                              ? const SizedBox(
                                  width: 20,
                                  height: 20,
                                  child: CircularProgressIndicator(strokeWidth: 2),
                                )
                              : const Icon(Icons.arrow_forward_ios, size: 16),
                          onTap: isLoading ? null : () => _connectToDevice(scanResult),
                        ),
                      );
                    }),
                  ],
                ),
            ],
          ),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _disconnect();
    super.dispose();
  }
}
