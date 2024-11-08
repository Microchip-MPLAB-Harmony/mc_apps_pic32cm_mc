---
title: Microchip MPLAB® Harmony 3 Motor Control Application Examples for PIC32CM MC family
nav_order: 1
has_children: true
has_toc: false
---

# Microchip MPLAB® Harmony 3 Motor Control Application Examples for PIC32CM MC family

MPLAB Harmony 3 is an extension of the MPLAB® ecosystem for creating
embedded firmware solutions for Microchip 32-bit SAM and PIC32 microcontroller
and microprocessor devices.  Refer to the following links for more information.
 - [Microchip 32-bit MCUs for Motor Control Applications](https://www.microchip.com/design-centers/motor-control-and-drive/control-products/32-bit-solutions)
 - [Microchip 32-bit MCUs](https://www.microchip.com/design-centers/32-bit)
 - [Microchip 32-bit MPUs](https://www.microchip.com/design-centers/32-bit-mpus)
 - [Microchip MPLAB X IDE](https://www.microchip.com/mplab/mplab-x-ide)
 - [Microchip MPLAB Harmony](https://www.microchip.com/mplab/mplab-harmony)
 - [Microchip MPLAB Harmony Pages](https://microchip-mplab-harmony.github.io/)

This repository contains the MPLAB® Harmony 3 Motor Control application exmaples for PIC32CM MC family. Users can use these examples as a reference for
developing their own motor control applications. Refer to the following links for release
notes and licensing information.

 - [Release Notes](./release_notes.md)
 - [MPLAB Harmony License](mplab_harmony_license.md)

## Contents Summary

| Folder     | Description                                               |
|------------|-----------------------------------------------------------|
| apps       | Demonstration applications for Motor Control              |
|||

## Documentation

Click [here](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MC_APPS_PIC32CM_MC_INTRODUCTION&redirect=true) to view the online documentation of code examples hosted in this repository.

To view the documentation offline, follow these steps:
 - Download the publication as a zip file from [here](https://onlinedocs.microchip.com/download/GUID-0F322B05-7084-4AFC-AFE1-790B85B05187?type=webhelp).
 - Extract the zip file into a folder.
 - Navigate to the folder and open **index.html** in a web browser of your choice.

## QSpin Example projects
The following example projects were created using Harmony QSpin Tool. For more details refer [motor_control repository](https://github.com/Microchip-MPLAB-Harmony/motor_control)

| Name | Description|Control Board|Inverter Board|
|:-----|:-----------|:------------|:-------------|
 [PMSM FOC using PLL Estimator](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MH3_pic32cm_mc_apps_pll_estimator&redirect=true)| Sensorless Field Oriented Control of PMSM using PLL estimator  | [PIC32CM MC00 Motor Control DIM](https://www.microchip.com/en-us/development-tool/ev61e63a)| [MCLV-48V-300W](https://www.microchip.com/en-us/development-tool/ev18h47a) |
|[PMSM FOC using Reduced Order Luenberger Observer](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MC_APPS_PIC32CM_MC_PMSM_FOC_USING_REDUCED_ORDER_LUENBERGER_OBSERVER&redirect=true)| Sensorless Field Oriented Control of PMSM using Reduced Order Luenberger Observer (ROLO) | [PIC32CM MC00 Motor Control DIM](https://www.microchip.com/en-us/development-tool/ev61e63a)| [MCLV-48V-300W](https://www.microchip.com/en-us/development-tool/ev18h47a) |

## Standalone ( Non-QSpin ) Motor Control Examples

These applications contain algorithm code and peripherals are configured using MCC. Configurations can be changed in userparam.h file. 


| Name | Description|Control Board|Inverter Board|
|:-----|:-----------|:------------|:-------------|
 | [BLDC Block Commutation using Hall Sensors](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MC_APPS_PIC32CM_MC_BLDC_BLOCK_COMMUTATION_HALL_SENSOR&redirect=true) | Block Commutation Control of BLDC motors using Hall Sensors |[PIC32CM MC00 Motor Control Plug-In-Module](https://www.microchip.com/developmenttools/ProductDetails/EV94F66A) |[dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) |
| [ACIM Open Loop V/Hz](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MC_APPS_PIC32CM_MC_ACIM_VHZ_CONTROL&redirect=true) | Open Loop V/Hz Control of ACIM |[PIC32CM MC00 Motor Control Plug-In-Module](https://www.microchip.com/developmenttools/ProductDetails/EV94F66A) |[dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3)|
 | [ BLDC sensorless Block Commutation with BEMF zero crossing detection](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MC_APPS_PIC32CM_MC_BLDC_BC_SENSORLESS&redirect=true) | Block Commutation Control of BLDC motors using Hall Sensors | [PIC32CM MC00 Motor Control DIM](https://www.microchip.com/en-us/development-tool/ev61e63a)| [MCLV-48V-300W](https://www.microchip.com/en-us/development-tool/ev18h47a) |

[![License](https://img.shields.io/badge/license-Harmony%20license-orange.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/blob/master/mplab_harmony_license.md)
[![Latest release](https://img.shields.io/github/release/Microchip-MPLAB-Harmony/mc_apps_pic32cm_mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/releases/latest)
[![Latest release date](https://img.shields.io/github/release-date/Microchip-MPLAB-Harmony/mc_apps_pic32cm_mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/releases/latest)
[![Commit activity](https://img.shields.io/github/commit-activity/y/Microchip-MPLAB-Harmony/mc_apps_pic32cm_mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/graphs/commit-activity)
[![Contributors](https://img.shields.io/github/contributors-anon/Microchip-MPLAB-Harmony/mc_apps_pic32cm_mc.svg)]()
____

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/user/MicrochipTechnology)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/microchip-technology)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/microchiptechnology/)
[![Follow us on Twitter](https://img.shields.io/twitter/follow/MicrochipTech.svg?style=social)](https://twitter.com/MicrochipTech)

[![](https://img.shields.io/github/stars/Microchip-MPLAB-Harmony/mc_apps_pic32cm_mc.svg?style=social)]()
[![](https://img.shields.io/github/watchers/Microchip-MPLAB-Harmony/mc_apps_pic32cm_mc.svg?style=social)]()