<!---

	Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
	This program and the accompanying materials are made available under
	the terms of the Bosch Internal Open Source License v4
	which accompanies this distribution, and is available at
	http://bios.intranet.bosch.com/bioslv4.txt

-->

# CC-AD/PJ-MP MapLoc-VFC <!-- omit in toc -->

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

This repository offers a catkin wrapper for [vfc](https://sourcecode.socialcoding.bosch.com/projects/VFC/repos/vfc/browse) package.
Unfortunately many Projects at Bosch use forks of the official vfc repository and do changes on them without upstreaming. This leads to problems by sharing code between projects. This repository was created due to the need of float16_storage_t type. This type is used in [radar_gen5]( https://sourcecode.socialcoding.bosch.com/projects/RS/repos/radar_gen5/browse)  to decode data gathered from radar sensors.

There is currently a pull-request for the float16_storage_t type in the official vfc repository. As soon as it merged to master, it will be possible to use it with radar_gen5 packages.

## Table of Contents  <!-- omit in toc -->

- [Getting Started](#getting-started)
- [About](#about)
  - [Contributors](#contributors)
  - [3rd Party Licenses](#3rd-party-licenses)
  - [Used Encryption](#used-encryption)
  - [License](#license)


## <a name="contributors">Contributors</a>

* [Gerd Zanker](https://connect.bosch.com/profiles/html/profileView.do?userid=325EEC38-C022-45D9-A278-ACAA9AC8685D)


### 3rd Party Licenses

You must mention all 3rd party licenses (e.g. OSS) licenses used by your
project here. Example:

| Name | License | Type |
|------|---------|------|
| [Apache Felix](http://felix.apache.org/) | [Apache 2.0 License](http://www.apache.org/licenses/LICENSE-2.0.txt) | Dependency

### Used Encryption

Declaration of the usage of any encryption (see BIOS Repository Policy §4.a).

### License

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

> Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
> This program and the accompanying materials are made available under
> the terms of the Bosch Internal Open Source License v4
> which accompanies this distribution, and is available at
> http://bios.intranet.bosch.com/bioslv4.txt

<!---

	Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
	This program and the accompanying materials are made available under
	the terms of the Bosch Internal Open Source License v4
	which accompanies this distribution, and is available at
	http://bios.intranet.bosch.com/bioslv4.txt

-->
