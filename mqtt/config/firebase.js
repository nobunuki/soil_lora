//const { app } = require('firebase-admin');
const {
  initializeApp,
  applicationDefault,
  cert,
}// = require('firebase-admin/app');
= require('firebase-admin/app');
const {
  //getFirestore,
  getDatabase,
  Timestamp,
  FieldValue,
} = require('firebase-admin/database');

const firebase_cert = require('./firebaseKey.json');

const firebaseConfig = {
  // ...
  // The value of `databaseURL` depends on the location of the database
  databaseURL: "https://summareconsensor-default-rtdb.firebaseio.com/",
  credential: cert(firebase_cert),
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
/*
initializeApp({
  credential: cert(firebase_cert),
});
*/
//const db = getFirestore();
const db = getDatabase(app);
module.exports = db;
/*
var admin = require("firebase-admin");

var serviceAccount = require("path/to/serviceAccountKey.json");

admin.initializeApp({
  credential: admin.credential.cert(serviceAccount),
  databaseURL: "https://ikanhias-f80a6-default-rtdb.firebaseio.com"
});*/