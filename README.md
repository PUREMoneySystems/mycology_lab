# mycology_lab


Automated Mycology (mushrooms) lab composed of 4 stages:
- Temperature Control
- Humidity Control
- Growth Chamber
- Fruit Chamber


Overall solution is to use 4 Arduinos to independently control each stage in the lab.  Each Arduino communicates with its peers over Wifi.  1 of the Arduinos acts as a gateway to communicate sensor readings and current state to a Google Spreadsheet.  The entire lab implements the idea of Runlevels from 0 (sleep) to 4 (full function, central control).  The Google Spreadsheet allows for data collection and an interface for building a lab execution profile.  Control system is modeled using Fuzzy logic in the Google Spreadsheet, with each Arduino implementing independent control logic if connectivity with peers is lost.
