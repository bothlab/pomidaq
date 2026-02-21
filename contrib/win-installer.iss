#define MyAppName      "PoMiDAQ"
#define MyAppPublisher "Matthias Klumpp"
#define MyAppURL       "https://github.com/bothlab/pomidaq"
#define MyAppExeName   "pomidaq.exe"
; These can be overridden from the command line: /DMyAppVersion=... /DDeployDir=...
#ifndef MyAppVersion
  #define MyAppVersion "0.1.0"
#endif
#ifndef DeployDir
  #define DeployDir "..\win64-deploy\PoMiDAQ-" + MyAppVersion
#endif

[Setup]
AppId={{FE994758-DCDE-40E4-BC6F-F626D1B5964A}}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}

; Installer exe metadata
VersionInfoVersion={#MyAppVersion}
VersionInfoDescription={#MyAppName} Installer
VersionInfoCompany={#MyAppPublisher}
VersionInfoCopyright=Copyright (C) 2020-2026 {#MyAppPublisher}

DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
; Ask for admin rights only when installing system-wide; allows per-user installs too
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog

LicenseFile={#DeployDir}\LICENSE.txt

OutputDir=output
OutputBaseFilename=PoMiDAQ-{#MyAppVersion}_Setup_win64
SetupIconFile={#DeployDir}\share\icons\pomidaq.ico
UninstallDisplayIcon={app}\bin\{#MyAppExeName}

Compression=lzma2/ultra64
SolidCompression=yes
ArchitecturesInstallIn64BitMode=x64compatible
WizardStyle=modern
; Require Windows 10 or later (Qt 6 minimum)
MinVersion=10.0
; Offer to close any running instance before installing/uninstalling
CloseApplications=yes
CloseApplicationsFilter=*.exe
RestartApplications=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "{#DeployDir}\bin\{#MyAppExeName}"; DestDir: "{app}\bin"; Flags: ignoreversion
Source: "{#DeployDir}\*"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs ignoreversion; Excludes: "bin\{#MyAppExeName},LICENSE.txt"
Source: "{#DeployDir}\LICENSE.txt"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#MyAppName}";         Filename: "{app}\bin\{#MyAppExeName}"
Name: "{commondesktop}\{#MyAppName}"; Filename: "{app}\bin\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\bin\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent
