# MacRM_Standard_2022
McMaster Robomaster Standard Infantry Robot Code for 2022 A&amp;M University Lone Star Competition

Developed based on "RoboMaster competition robot 2020 self-assembly version A". Refer to user manuals posted on
> "File download: [RoboMaster Competition Robot 2020 Self-Assembled Version A Type Related Documents](https://rm-static.djicdn.com/documents/26898/57bcb7163d7bd1575980335867998835.7z)"

on 
> RM's Official website: https://www.robomaster.com/zh-CN/resource/pages/announcement/1081.

# Firmware Environment
- Type C Development Board
- FreeRTOS

# Electrical Assembly
## INFANTRY_2 Diagram
``` mermaid
---
title: 2022 MacFalcons INFANTRY_2 Electrical Assembly
---
%% Naming convention: first prefix is module name
flowchart
    subgraph Chassis
        subgraph CV["Jetson Nano"]
            CV-DcJack["19VDC Power Input"]
            subgraph CV-USB-Camera["Camera USB"]
                CV-USB-Camera-5V["5V"]
                CV-USB-Camera-DataH["D+"]
                CV-USB-Camera-DataL["D-"]
                CV-USB-Camera-Gnd["Gnd"]
            end
            subgraph CV-CtrltoCv-USBtoTTL["USB-to-TTL to Control"]
                CV-CtrltoCv-Rx["Rx"]
                CV-CtrltoCv-Tx["Tx"]
                CV-CtrltoCv-Gnd["Gnd"]
            end
        end

        subgraph PMM["Ref Power Management Module (PMM)"]
            PMM-Battery["XT60-Battery"]
            PMM-MiniPC["XT30-MiniPC"]
            PMM-Chassis["XT30-Chassis"]
            subgraph PMM-Gimbal["XT30-Gimbal"]
                PMM-Gimbal-24V["24VDC"]
                PMM-Gimbal-Gnd["Gnd"]
            end
            subgraph PMM-AmmoBooster["XT30-AmmoBooster"]
                PMM-AmmoBooster-24V["24VDC"]
                PMM-AmmoBooster-Gnd["Gnd"]
            end
            subgraph PMM-M8["Ref M8 Cable"]
                PMM-M8-12V["Ref M8 VCC (Blue, 12V)"]
                PMM-M8-CANH["Ref M8 CANH"]
                PMM-M8-CANL["Ref M8 CANL"]
                PMM-M8-Gnd["Gnd (Brown)"]
            end
            subgraph UserPort["User Serial Port"]
                PMM-CtrltoRef-Rx["Rx"]
                PMM-CtrltoRef-Tx["Tx"]
                PMM-CtrltoRef-Gnd["Gnd"]
            end
        end

        subgraph ExtBoard1["ESC Center Board-1"]
            EB1-XT30["XT30 (7 ports)"]
            EB1-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB1-CAN-4Pin["CAN-4Pin"]
            EB1-XT60["XT60"]
        end
        
        Chassis-C620["Four C620 controllers"]---Chassis-M3508["Four M3508 motors"]

        %% Connections
        PMM-Chassis-->EB1-XT60["XT60"]
        Battery["Battery TB47S (24V)"]-->PMM-Battery       
        PMM-MiniPC-->BuckConvertor["24V-to-19V Buck Convertor"]-->CV-DcJack
        EB1-CAN-2Pin & EB1-XT30===Chassis-C620
    end

    subgraph SlipRing
        subgraph Ref-UART["Ref UART"]
            SR-CtrltoRef-Rx["Ref UART Rx"]
            SR-CtrltoRef-Tx["Ref UART Tx"]
        end
        subgraph CtrltoCv-UART["CV UART"]
            SR-CtrltoCv-Rx["CV UART Rx"]
            SR-CtrltoCv-Tx["CV UART Tx"]
        end
        subgraph Ref-M8["Ref M8 Cable Pins"]
            SR-M8-12V["Ref M8 VCC"]
            SR-M8-CANH["Ref M8 CANH"]
            SR-M8-CANL["Ref M8 CANL"]
        end
        subgraph Camera-USB["Camera USB"]
            SR-USB-Camera-DataH["Camera USB D+"]
            SR-USB-Camera-DataL["Camera USB D-"]
        end
        subgraph SR-ChassisCAN["Chassis CAN"]
            SR-ChassisCANH["CAN High"]
            SR-ChassisCANL["CAN Low"]
        end
        SR-AmmoBooster-24V["AmmoBooster Power"]
        SR-Gimbal-24V["Gimbal Power"]
        SR-Gnd["Common Gnd
            Some signal gnd don't have to be connected,
            but would better be connected to shorten gnd path"]
    end

    subgraph Gimbal
        subgraph ExtBoard2["ESC Center Board-2"]
            EB2-XT30["XT30 (7 ports)"]
            EB2-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB2-CAN-4Pin["CAN-4Pin"]
            EB2-XT60["XT60"]
        end

        subgraph CtrlBoardC["Type C Control Board"]
            subgraph CtrltoCv-UART2["UART2 for CV (USART1 in code)"]
                Ctrl-CtrltoCv-Rx["Rx"]
                Ctrl-CtrltoCv-Tx["Tx"]
                Ctrl-CtrltoCv-Gnd["Gnd"]
            end
            subgraph CtrltoRef-UART1["UART1 for Ref (USART6 in code)"]
                Ctrl-CtrltoRef-Rx["Rx"]
                Ctrl-CtrltoRef-Tx["Tx"]
                Ctrl-CtrltoRef-Gnd["Gnd"]
            end
            subgraph Port-5V-Out-2Pin["5V output port"]
                Ctrl-5V-VCC["5V"]
                Ctrl-5V-Gnd["Gnd"]
            end
            CAN1-Chassis["Chassis CAN (CAN1)"]
            CAN2-Gimbal["Gimbal CAN (CAN2)"]
        end

        RefGim-M8["Ref Parts wired by Daisy Chain of M8 cables"]

        subgraph Camera["Camera"]
            Camera-DataH["D+"]
            Camera-DataL["D-"]
            Camera-Gnd["Gnd"]
            Camera-5V["5V"]
        end

        %% Connections
        Ctrl-5V-VCC---Camera-5V
        CAN2-Gimbal===EB2-CAN-4Pin
    end

    PMM-CtrltoRef-Rx---SR-CtrltoRef-Rx---Ctrl-CtrltoRef-Tx
    PMM-CtrltoRef-Tx---SR-CtrltoRef-Tx---Ctrl-CtrltoRef-Rx
    PMM-CtrltoRef-Gnd---SR-Gnd---Ctrl-CtrltoRef-Gnd

    PMM-M8-12V---SR-M8-12V---RefGim-M8
    PMM-M8-CANH===SR-M8-CANH===RefGim-M8
    PMM-M8-CANL===SR-M8-CANL===RefGim-M8
    PMM-M8-Gnd---SR-Gnd---RefGim-M8

    PMM-AmmoBooster-24V---SR-AmmoBooster-24V
    PMM-AmmoBooster-Gnd---SR-Gnd

    PMM-Gimbal-24V---SR-Gimbal-24V
    PMM-Gimbal-Gnd---SR-Gnd

    CV-CtrltoCv-Rx --- SR-CtrltoCv-Rx --- Ctrl-CtrltoCv-Tx
    CV-CtrltoCv-Tx --- SR-CtrltoCv-Tx --- Ctrl-CtrltoCv-Rx
    CV-CtrltoCv-Gnd --- SR-Gnd --- Ctrl-CtrltoCv-Gnd

    CV-USB-Camera-DataH---SR-USB-Camera-DataH---Camera-DataH
    CV-USB-Camera-DataL---SR-USB-Camera-DataL---Camera-DataL
    CV-USB-Camera-Gnd---SR-Gnd---Camera-Gnd

    EB1-CAN-2Pin---SR-ChassisCAN---CAN1-Chassis
```
## INFANTRY_3 Diagram
``` mermaid
---
title: 2022 MacFalcons INFANTRY_3 Electrical Assembly
---
%% Naming convention: first prefix is module name
flowchart
    subgraph Chassis
        subgraph CV["Jetson Nano"]
            CV-DcJack["19VDC Power Input"]
            subgraph CV-USB-Camera["Camera USB"]
                CV-USB-Camera-5V["5V"]
                CV-USB-Camera-DataH["D+"]
                CV-USB-Camera-DataL["D-"]
                CV-USB-Camera-Gnd["Gnd"]
            end
            subgraph CV-CtrltoCv-USBtoTTL["USB-to-TTL to Control"]
                CV-CtrltoCv-Rx["Rx"]
                CV-CtrltoCv-Tx["Tx"]
                CV-CtrltoCv-Gnd["Gnd"]
            end
        end

        subgraph PMM["Ref Power Management Module (PMM)"]
            PMM-Battery["XT60-Battery"]
            PMM-MiniPC["XT30-MiniPC"]
            PMM-Chassis["XT30-Chassis"]
            subgraph PMM-Gimbal["XT30-Gimbal"]
                PMM-Gimbal-24V["24VDC"]
                PMM-Gimbal-Gnd["Gnd"]
            end
            subgraph PMM-AmmoBooster["XT30-AmmoBooster"]
                PMM-AmmoBooster-24V["24VDC"]
                PMM-AmmoBooster-Gnd["Gnd"]
            end
            subgraph PMM-M8["Ref M8 Cable"]
                PMM-M8-12V["Ref M8 VCC (Blue, 12V)"]
                PMM-M8-CANH["Ref M8 CANH"]
                PMM-M8-CANL["Ref M8 CANL"]
                PMM-M8-Gnd["Gnd (Brown)"]
            end
            subgraph UserPort["User Serial Port"]
                PMM-CtrltoRef-Rx["Rx"]
                PMM-CtrltoRef-Tx["Tx"]
                PMM-CtrltoRef-Gnd["Gnd"]
            end
        end

        subgraph ExtBoard1["ESC Center Board-1"]
            EB1-XT30["XT30 (7 ports)"]
            EB1-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB1-CAN-4Pin["CAN-4Pin"]
            EB1-XT60["XT60"]
        end
        
        Chassis-C620["Four C620 controllers"]---Chassis-M3508["Four M3508 motors"]

        %% Connections
        PMM-Chassis-->EB1-XT60["XT60"]
        Battery["Battery TB47S (24V)"]-->PMM-Battery       
        PMM-MiniPC-->BuckConvertor["24V-to-19V Buck Convertor"]-->CV-DcJack
        EB1-CAN-2Pin & EB1-XT30===Chassis-C620
    end

    subgraph SlipRing
        subgraph Ref-UART["Ref UART"]
            SR-CtrltoRef-Rx["Ref UART Rx"]
            SR-CtrltoRef-Tx["Ref UART Tx"]
        end
        subgraph CtrltoCv-UART["CV UART"]
            SR-CtrltoCv-Rx["CV UART Rx"]
            SR-CtrltoCv-Tx["CV UART Tx"]
        end
        subgraph Ref-M8["Ref M8 Cable Pins"]
            SR-M8-12V["Ref M8 VCC"]
            SR-M8-CANH["Ref M8 CANH"]
            SR-M8-CANL["Ref M8 CANL"]
        end
        subgraph Camera-USB["Camera USB"]
            SR-USB-Camera-DataH["Camera USB D+"]
            SR-USB-Camera-DataL["Camera USB D-"]
        end
        subgraph SR-ChassisCAN["Chassis CAN"]
            SR-ChassisCANH["CAN High"]
            SR-ChassisCANL["CAN Low"]
        end
        SR-AmmoBooster-24V["AmmoBooster Power"]
        SR-Gimbal-24V["Gimbal Power"]
        SR-Gnd["Common Gnd
            Some signal gnd don't have to be connected,
            but would better be connected to shorten gnd path"]
    end

    subgraph Gimbal
        subgraph ExtBoard2["ESC Center Board-2"]
            EB2-XT30["XT30 (7 ports)"]
            EB2-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB2-CAN-4Pin["CAN-4Pin"]
            EB2-XT60["XT60"]
        end

        subgraph CtrlBoardC["Type C Control Board"]
            subgraph CtrltoCv-UART2["UART2 for CV (USART1 in code)"]
                Ctrl-CtrltoCv-Rx["Rx"]
                Ctrl-CtrltoCv-Tx["Tx"]
                Ctrl-CtrltoCv-Gnd["Gnd"]
            end
            subgraph CtrltoRef-UART1["UART1 for Ref (USART6 in code)"]
                Ctrl-CtrltoRef-Rx["Rx"]
                Ctrl-CtrltoRef-Tx["Tx"]
                Ctrl-CtrltoRef-Gnd["Gnd"]
            end
            subgraph Port-5V-Out-2Pin["5V output port"]
                Ctrl-5V-VCC["5V"]
                Ctrl-5V-Gnd["Gnd"]
            end
            CAN1-Chassis["Chassis CAN (CAN1)"]
            CAN2-Gimbal["Gimbal CAN (CAN2)"]
        end

        RefGim-M8["Ref Parts wired by Daisy Chain of M8 cables"]

        subgraph Camera["Camera"]
            Camera-DataH["D+"]
            Camera-DataL["D-"]
            Camera-Gnd["Gnd"]
            Camera-5V["5V"]
        end

        %% Connections
        Ctrl-5V-VCC---Camera-5V
        CAN2-Gimbal===EB2-CAN-4Pin
    end

    PMM-CtrltoRef-Rx---SR-CtrltoRef-Rx---Ctrl-CtrltoRef-Tx
    PMM-CtrltoRef-Tx---SR-CtrltoRef-Tx---Ctrl-CtrltoRef-Rx
    PMM-CtrltoRef-Gnd---SR-Gnd---Ctrl-CtrltoRef-Gnd

    PMM-M8-12V---SR-M8-12V---RefGim-M8
    PMM-M8-CANH===SR-M8-CANH===RefGim-M8
    PMM-M8-CANL===SR-M8-CANL===RefGim-M8
    PMM-M8-Gnd---SR-Gnd---RefGim-M8

    PMM-AmmoBooster-24V---SR-AmmoBooster-24V
    PMM-AmmoBooster-Gnd---SR-Gnd

    PMM-Gimbal-24V---SR-Gimbal-24V
    PMM-Gimbal-Gnd---SR-Gnd

    CV-CtrltoCv-Rx --- SR-CtrltoCv-Rx --- Ctrl-CtrltoCv-Tx
    CV-CtrltoCv-Tx --- SR-CtrltoCv-Tx --- Ctrl-CtrltoCv-Rx
    CV-CtrltoCv-Gnd --- SR-Gnd --- Ctrl-CtrltoCv-Gnd

    CV-USB-Camera-DataH---SR-USB-Camera-DataH---Camera-DataH
    CV-USB-Camera-DataL---SR-USB-Camera-DataL---Camera-DataL
    CV-USB-Camera-Gnd---SR-Gnd---Camera-Gnd

    EB1-CAN-2Pin---SR-ChassisCAN---CAN1-Chassis
```

## SENTRY_1 Diagram
``` mermaid
---
title: 2022 MacFalcons SENTRY_1 Electrical Assembly
---
%% Naming convention: first prefix is module name
flowchart
    subgraph Chassis
        subgraph CV["Jetson Nano"]
            CV-DcJack["19VDC Power Input"]
            subgraph CV-USB-Camera["Camera USB"]
                CV-USB-Camera-5V["5V"]
                CV-USB-Camera-DataH["D+"]
                CV-USB-Camera-DataL["D-"]
                CV-USB-Camera-Gnd["Gnd"]
            end
            subgraph CV-CtrltoCv-USBtoTTL["USB-to-TTL to Control"]
                CV-CtrltoCv-Rx["Rx"]
                CV-CtrltoCv-Tx["Tx"]
                CV-CtrltoCv-Gnd["Gnd"]
            end
        end

        subgraph PMM["Ref Power Management Module (PMM)"]
            PMM-Battery["XT60-Battery"]
            PMM-MiniPC["XT30-MiniPC"]
            PMM-Chassis["XT30-Chassis"]
            subgraph PMM-Gimbal["XT30-Gimbal"]
                PMM-Gimbal-24V["24VDC"]
                PMM-Gimbal-Gnd["Gnd"]
            end
            subgraph PMM-AmmoBooster["XT30-AmmoBooster"]
                PMM-AmmoBooster-24V["24VDC"]
                PMM-AmmoBooster-Gnd["Gnd"]
            end
            subgraph PMM-M8["Ref M8 Cable"]
                PMM-M8-12V["Ref M8 VCC (Blue, 12V)"]
                PMM-M8-CANH["Ref M8 CANH"]
                PMM-M8-CANL["Ref M8 CANL"]
                PMM-M8-Gnd["Gnd (Brown)"]
            end
            subgraph UserPort["User Serial Port"]
                PMM-CtrltoRef-Rx["Rx"]
                PMM-CtrltoRef-Tx["Tx"]
                PMM-CtrltoRef-Gnd["Gnd"]
            end
        end

        subgraph ExtBoard1["ESC Center Board-1"]
            EB1-XT30["XT30 (7 ports)"]
            EB1-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB1-CAN-4Pin["CAN-4Pin"]
            EB1-XT60["XT60"]
        end
        
        Chassis-C620["Four C620 controllers"]---Chassis-M3508["Four M3508 motors"]

        %% Connections
        PMM-Chassis-->EB1-XT60["XT60"]
        Battery["Battery TB47S (24V)"]-->PMM-Battery       
        PMM-MiniPC-->BuckConvertor["24V-to-19V Buck Convertor"]-->CV-DcJack
        EB1-CAN-2Pin & EB1-XT30===Chassis-C620
    end

    subgraph SlipRing
        subgraph Ref-UART["Ref UART"]
            SR-CtrltoRef-Rx["Ref UART Rx"]
            SR-CtrltoRef-Tx["Ref UART Tx"]
        end
        subgraph CtrltoCv-UART["CV UART"]
            SR-CtrltoCv-Rx["CV UART Rx"]
            SR-CtrltoCv-Tx["CV UART Tx"]
        end
        subgraph Ref-M8["Ref M8 Cable Pins"]
            SR-M8-12V["Ref M8 VCC"]
            SR-M8-CANH["Ref M8 CANH"]
            SR-M8-CANL["Ref M8 CANL"]
        end
        subgraph Camera-USB["Camera USB"]
            SR-USB-Camera-DataH["Camera USB D+"]
            SR-USB-Camera-DataL["Camera USB D-"]
        end
        subgraph SR-ChassisCAN["Chassis CAN"]
            SR-ChassisCANH["CAN High"]
            SR-ChassisCANL["CAN Low"]
        end
        SR-AmmoBooster-24V["AmmoBooster Power"]
        SR-Gimbal-24V["Gimbal Power"]
        SR-Gnd["Common Gnd
            Some signal gnd don't have to be connected,
            but would better be connected to shorten gnd path"]
    end

    subgraph Gimbal
        subgraph ExtBoard2["ESC Center Board-2"]
            EB2-XT30["XT30 (7 ports)"]
            EB2-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB2-CAN-4Pin["CAN-4Pin"]
            EB2-XT60["XT60"]
        end

        subgraph CtrlBoardC["Type C Control Board"]
            subgraph CtrltoCv-UART2["UART2 for CV (USART1 in code)"]
                Ctrl-CtrltoCv-Rx["Rx"]
                Ctrl-CtrltoCv-Tx["Tx"]
                Ctrl-CtrltoCv-Gnd["Gnd"]
            end
            subgraph CtrltoRef-UART1["UART1 for Ref (USART6 in code)"]
                Ctrl-CtrltoRef-Rx["Rx"]
                Ctrl-CtrltoRef-Tx["Tx"]
                Ctrl-CtrltoRef-Gnd["Gnd"]
            end
            subgraph Port-5V-Out-2Pin["5V output port"]
                Ctrl-5V-VCC["5V"]
                Ctrl-5V-Gnd["Gnd"]
            end
            CAN1-Chassis["Chassis CAN (CAN1)"]
            CAN2-Gimbal["Gimbal CAN (CAN2)"]
        end

        RefGim-M8["Ref Parts wired by Daisy Chain of M8 cables"]

        subgraph Camera["Camera"]
            Camera-DataH["D+"]
            Camera-DataL["D-"]
            Camera-Gnd["Gnd"]
            Camera-5V["5V"]
        end

        %% Connections
        Ctrl-5V-VCC---Camera-5V
        CAN2-Gimbal===EB2-CAN-4Pin
    end

    PMM-CtrltoRef-Rx---SR-CtrltoRef-Rx---Ctrl-CtrltoRef-Tx
    PMM-CtrltoRef-Tx---SR-CtrltoRef-Tx---Ctrl-CtrltoRef-Rx
    PMM-CtrltoRef-Gnd---SR-Gnd---Ctrl-CtrltoRef-Gnd

    PMM-M8-12V---SR-M8-12V---RefGim-M8
    PMM-M8-CANH===SR-M8-CANH===RefGim-M8
    PMM-M8-CANL===SR-M8-CANL===RefGim-M8
    PMM-M8-Gnd---SR-Gnd---RefGim-M8

    PMM-AmmoBooster-24V---SR-AmmoBooster-24V
    PMM-AmmoBooster-Gnd---SR-Gnd

    PMM-Gimbal-24V---SR-Gimbal-24V
    PMM-Gimbal-Gnd---SR-Gnd

    CV-CtrltoCv-Rx --- SR-CtrltoCv-Rx --- Ctrl-CtrltoCv-Tx
    CV-CtrltoCv-Tx --- SR-CtrltoCv-Tx --- Ctrl-CtrltoCv-Rx
    CV-CtrltoCv-Gnd --- SR-Gnd --- Ctrl-CtrltoCv-Gnd

    CV-USB-Camera-DataH---SR-USB-Camera-DataH---Camera-DataH
    CV-USB-Camera-DataL---SR-USB-Camera-DataL---Camera-DataL
    CV-USB-Camera-Gnd---SR-Gnd---Camera-Gnd

    EB1-CAN-2Pin---SR-ChassisCAN---CAN1-Chassis
```

# Branches
- test_no_ref: test all other features without referee system by ignoring ref errrors