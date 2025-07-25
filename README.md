Project AIROT (Automated Intercept and Rendevous of an Orbital Target) is the code and crafts used learning how to automate the docking of two crafts in the game Kerbal Space Program  
A self-imposed limit for this project is a ban on any autopilot feature not availabe to a ingame player (SAS modes are alowed) nesesitating the creation of custom control loops.  

# Mods Used  
  
As well as playing with the two DLCs for KSP (Making History and Breaking Ground) I play with the folowing mods that do not change any of the core mechanics of the game managed by the CKAN mod manager.  
* Firefly (Visual, reentry effects)  
* Harmony 2 (Automaticly added by the CKAN mod manager)  
* Kerbal Engineer Redux (Technical, provides an overlay of more in-flight information to supplement data provided from the Aero-Gui)  
* Kopernicus Planetary System Modifier (Automaticly added by the CKAN mod manager)  
* kRPC: Remote Procedure Call Server (Technical, the mod that makes this project posible by alowing user writen code to control crafts in game)  
* KSP Comunity fixes (Technical, fixes common bugs among other things)  
* ModularFlightIntegrator (Automaticly added by the CKAN mod manager)  
* Module Manager (Technical, Automaticly added by the CKAN mod manager)  
* Paralax (Visual)  
* Parallax - Stock Planet Textures (Visual)  
* Parallax- Stock Scatter Textures (Visual)  
* Scatterer (Visual)  
* Scatterer Default Config (Visual)  
* Scatterer Sunflare (Visual)  

# Process  
The prosses for the development of this project started with the [krpcTest file](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/krpcTest.py) as a test to see if I had installed the KRPC mod correctly, I then moved on to the [Simple Solid Craft](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/Crafts/Simple%20Solid.craft) and it's companion program [SimpleSolid.py](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/simpleSolidTest.py). This was made to learn the  
basics of how to interface my code with the game and learn how a few of the methods workd from the mod. This was a simple straight up and down flight with a crew capsule seperating at apoapsis and having a main chute deploy to slow the craft down to a safe speed for landing.  
  
The next step in this program was to automate a suborbital flight with a propper flight profile nessesitating the change in attitude of the craft. The [craft](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/Crafts/Simple%20Liquid.craft) and it's control program [SimpleLiquid.py](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/simpleLiquidTest.py) that required the implementation of a circle queue class housed  
[here](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/CircleQueue.py). The circle queue was nessasary due to multiple threads reliant on a single condition.  

The third step in this program is to automate [this craft](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/Crafts/Crewed%20Orbital%20Rendezvous%20Craft.craft) into orbit and then calculate the nessasary maneuvers to intercept and  
then rendevous to a [space station](https://github.com/RocketsAndRedstone/Project-AIROT/blob/main/Crafts/Agena%20Space%20Station.craft) placed into a 100km by 100km orbit using cheats. Some of the calculations and logic for this  
procedure come from [NASA document 19660023038](https://ntrs.nasa.gov/api/citations/19660023038/downloads/19660023038.pdf).
