const db = require('./config/firebase');
const mqtt = require("mqtt");
const { FieldValue, Firebase } = require('firebase-admin/database');

const host = 'au1.cloud.thethings.network'
const port = '1883'
const clientId = `mqtt_${Math.random().toString(16).slice(3)}`
const connectUrl = `mqtt://${host}:${port}`

const client = mqtt.connect(connectUrl, {
    clientId,
    clean:true,
    connectTimeout:4000,
    username:'summarecon-sensor@ttn',
    password:'NNSXS.2LABVHM2CI5JZ6UPPEQEVPOOKDKNEDAD5BJYFWQ.VEZUY2PPR3SCT2TR3HCWVCEU7KMOMSVM7JZTSJHA2SY4PIPANS3A',
    reconnectPeriod:1000,
})

let end_devices = [
    "v3/summarecon-sensor@ttn/devices/sensor-soil-1/up"
];

console.log("RUN ....")

client.on('connect', () => {
    client.subscribe(end_devices, {
        qos: 0,
        retain: false
    }, (err, granted) => {
        console.log(granted);
    });
    console.log("connected : " + client.connected)
});

client.on("reconnect", () => {
    console.log("reconnect ....")
});

client.on("error", e => {
    console.log("error: " + e.message);
});

client.on('message', async function(topic, payload, packet) {
    const dataParse = JSON.parse(payload.toString())
    console.log('Data Parse: ' + dataParse)
    //console.log('Datetime', now())

/*
    const iaq = dataParse.uplink_message.decoded_payload.iaq;
    let condition = "";

    const checkCondition = (iaq) => {
        if (iaq > 0 && iaq <= 50) {
            condition = "good"
        } else if (iaq > 50 && iaq <= 100) {
            condition = "moderate";
        } else if (iaq > 100 && iaq <= 200) {
            condition = "unhealthy"
        } else if (iaq > 200 && iaq <= 300) {
            condition = "very unhealthy"
        } else if (iaq > 300 && iaq <= 500) {
            condition = "hazardous"
        } else {
            condition = "good"
        }
    }
    checkCondition(iaq)
*/
    const data = {
       // condition: condition,
      //  created_at: FieldValue.serverTimestamp(),
        data_result: {
            soilmoisture: dataParse.uplink_message.decoded_payload.SoilMoisture,
            soiltemperature: dataParse.uplink_message.decoded_payload.SoilTemperature,
            airtemperature: dataParse.uplink_message.decoded_payload.AirTemperature,
            airpressure: dataParse.uplink_message.decoded_payload.AirPressure,
            airhumidity: dataParse.uplink_message.decoded_payload.AirHumidity,
            gasresistance: dataParse.uplink_message.decoded_payload.GasResistance,
            receivedAt: dataParse.received_at
        }
       // device_id: db.collection('device').doc(dataParse.end_device_ids.device_id)
    }
    console.log(data)
/*
    const sensorsRef = db.collection('kolam1');
    const saveData = async() => {
        const addData = await sensorsRef.add(data)
        console.log(addData.id)
    }
    saveData()
*/


const sensorsRef = db.refFromURL('https://summareconsensor-default-rtdb.firebaseio.com/');
const saveData = async() => {
    const addData = await sensorsRef.set({
        sensor: {
            soilmoisture: dataParse.uplink_message.decoded_payload.SoilMoisture,
            soiltemperature: dataParse.uplink_message.decoded_payload.SoilTemperature,
            airtemperature: dataParse.uplink_message.decoded_payload.AirTemperature,
            airpressure: dataParse.uplink_message.decoded_payload.AirPressure,
            airhumidity: dataParse.uplink_message.decoded_payload.AirHumidity,
            gasresistance: dataParse.uplink_message.decoded_payload.GasResistance,
            receivedAt: dataParse.received_at
          }
          })
    //console.log(addData.id)
}
saveData()
/*sensorsRef.set({
  alanisawesome: {
    date_of_birth: 'June 23, 1912',
    full_name: 'Alan Turing'
  },
  gracehop: {
    date_of_birth: 'December 9, 1906',
    full_name: 'Grace Hopper'
  }
});*/
})