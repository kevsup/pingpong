import React, { useState, Component } from 'react';
import { StyleSheet, Button, View, SafeAreaView, Text, Alert, TextInput } from 'react-native';

// style could use some work later - decompose

const App = () => (
  <SafeAreaView style={styles.container}>
    <ActionButtons />
  </SafeAreaView>
);

const Separator = () => (
  <View style={styles.separator} />
);

const ActionButtons = () => {
  const tau = '\u03C4'
  const url = 'https://crackerjack-dinosaur-4740.dataplicity.io/pingpong/'
  const MIN_PERIOD = 1500
  const DEFAULT_PERIOD = 3000
  const DEFAULT_PATTERN = 'SWEEP'
  const DEFAULT_SPIN = 'BACKSPIN'
  const [period, setPeriod] = useState(DEFAULT_PERIOD)
  const [text, setText] = useState('')
  const [running, setRunning] = useState(true)
  const [shutdown, setShutdown] = useState(false)
  const [initialize, setInitialize] = useState(true)
  const [shotPattern, setShotPattern] = useState(DEFAULT_PATTERN)
  const [spin, setSpin] = useState(DEFAULT_SPIN)

  if (initialize) {
    const suffix = 'toggle/'
    fetch(url + suffix)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson); 
      setRunning(responseJson.running)
      })
    .catch((error) => { console.error(error) });

    const timeSuffix = 'time/'
    fetch(url + timeSuffix)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson); 
      setPeriod(responseJson.delay)
      })
    .catch((error) => { console.error(error) });

    const shotSuffix = 'shot/'
    fetch(url + shotSuffix)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson); 
      setShotPattern(responseJson.shot)
      })
    .catch((error) => { console.error(error) });

    const spinSuffix = 'spin/'
    fetch(url + spinSuffix)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson); 
      setSpin(responseJson.spin)
      })
    .catch((error) => { console.error(error) });

    setInitialize(false)
  }

  // --------- PRIVATE HELPER FUNCTIONS -------------

  const buttonPress = (newPeriod) => {
    if (newPeriod.localeCompare('') != 0 && !isNaN(newPeriod)) {
      if (newPeriod >= MIN_PERIOD) {
    speedPostCall(newPeriod)
      } else {
    Alert.alert('Minimum period is ' + MIN_PERIOD)
      }
    } else {
      Alert.alert('Input a valid number')
    }
  }

  const spinPostCall = (newSpin) => {
    const suffix = 'spin/'
    var form = new FormData()
    form.append('spin', newSpin)
    let data = {
      method: 'POST',
      body: form
    }
    fetch(url + suffix, data)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson);
      setSpin(responseJson.spin)
      Alert.alert('Spin is now ' + newSpin)
      })
    .catch((error) => { 
      console.error(error);
      Alert.alert('Error changing spin')
      });
  }

  const shotPostCall = (shotLocation) => {
    const suffix = 'shot/'
    var form = new FormData()
    form.append('location', shotLocation)
    let data = {
      method: 'POST',
      body: form
    }
    fetch(url + suffix, data)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson);
      setShotPattern(responseJson.shot)
      Alert.alert('Shooting pattern is now ' + shotLocation)
      })
    .catch((error) => { 
      console.error(error);
      Alert.alert('Error changing shooting pattern')
      });
  }

  const speedPostCall = (newPeriod) => {
    const suffix = 'time/'
    var form = new FormData()
    form.append('delay', newPeriod)
    let data = {
      method: 'POST',
      body: form
    }
    fetch(url + suffix, data)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson);
      setPeriod(newPeriod)
      Alert.alert('Period is now ' + newPeriod)
      })
    .catch((error) => { 
      console.error(error);
      Alert.alert('Error changing shooting period')
      });
  }

  const toggleCall = (prevState) => {
    const starting = !prevState
    const suffix = 'toggle/'
    var form = new FormData()
    form.append('action', starting ? 'start' : 'stop')
    let data = {
      method: 'POST',
      body: form
    }
    fetch(url + suffix, data)
    .then((response) => response.json())
    .then((responseJson) => { 
      console.log('response:', responseJson);
      setRunning(starting);
      Alert.alert('Robot toggled'); 
      })
    .catch((error) => { 
      console.error(error);
      Alert.alert('Error toggling robot'); 
      });
  }

   function shutdownPi() {
    const suffix = 'shutdown/'
    fetch(url + suffix)
    .then((response) => response.json())
    .then((responseJson) => {
        console.log('response:', responseJson);
        setShutdown(true);  
        Alert.alert('Raspberry Pi shutdown');
    })
    .catch((error) => {
        console.error(error);
        Alert.alert('Error shutting down');
    });
  }

  return (


    <View>


      <Text style={styles.title}>
    Select spin:
      </Text>

      <View style={styles.button_row}>

        <Button
            title="TOPSPIN"
            disabled = {shutdown}
            color= {spin.localeCompare('TOPSPIN') == 0 ? "#0000FF" : "#87A6CE"}
            onPress={() => spinPostCall('TOPSPIN')}
        />
        <Button
            title="BACKSPIN"
            disabled = {shutdown}
            color= {spin.localeCompare('BACKSPIN') == 0 ? "#0000FF" : "#87A6CE"}
            onPress={() => spinPostCall('BACKSPIN')}
        />
      </View>

      <Separator />

      <Text style={styles.title}>
    Select shooting pattern:
      </Text>

      <View style={styles.button_row}>

        <Button
            title="FOREHAND"
            disabled = {shutdown}
            color= {shotPattern.localeCompare('FOREHAND') == 0 ? "#0000FF" : "#87A6CE"}
            onPress={() => shotPostCall('FOREHAND')}
        />
        <Button
            title="BACKHAND"
            disabled = {shutdown}
            color= {shotPattern.localeCompare('BACKHAND') == 0 ? "#0000FF" : "#87A6CE"}
            onPress={() => shotPostCall('BACKHAND')}
        />
        <Button
            title="SWEEP"
            disabled = {shutdown}
            color= {shotPattern.localeCompare('FOREHAND') != 0 && shotPattern.localeCompare('BACKHAND') != 0 ? "#0000FF" : "#87A6CE"}
            onPress={() => shotPostCall('sweep')}
        />
      </View>

      <Separator />

      <Text style={styles.title}>
        Current shooting period: {tau} = {period} milliseconds
      </Text>

      <TextInput
        style={styles.title}
        placeholder="Enter new period here"
        onChangeText={text => setText(text)}
        defaultValue={text}
      />

      <Button
        title="CHANGE SHOOTING SPEED"
        disabled = {shutdown}
        onPress={() => buttonPress(text)}
      />

      <Separator />

      <Text style={styles.title}>
        Press to {running ? "stop" : "start" } robot 
      </Text>
      <Button
        title= {running ? "STOP ROBOT" : "START ROBOT"}
        color= {running ? "#E94A27" : "#41E927"}
        disabled = {shutdown}
        onPress={() => toggleCall(running)}
      />
      
      <Separator />
        
      <Text style={styles.title_italic}>
        Press to shutdown Raspberry Pi{'\n'}(DO NOT just pull its plug first!)
      </Text>

      <Button
        title= "shut down" 
        color= "#E94A27"
        disabled = {shutdown}
        onPress={() => shutdownPi()}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    marginHorizontal: 16,
  },
  title: {
    textAlign: 'center',
    marginVertical: 8,
    fontSize: 24,
  },
  title_italic: {
    textAlign: 'center',
    marginVertical: 8,
    fontSize: 18,
    fontStyle: 'italic',
  },
  separator: {
    marginVertical: 8,
    borderBottomColor: '#737373',
    borderBottomWidth: StyleSheet.hairlineWidth,
  },
  button_row: {
    flexDirection: 'row',
    justifyContent: 'center'
  }
});

export default App;
