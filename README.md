# Git usage
At some point, we'll introduce a feature-complete git system. For now, let's keep things simple but comply with the most important rules. This keeps repos running and prevents damage and headaches, so please follow these rules:

- Protect the main-branch: never push to the main branch directly. If you want to add a new feature (f. ex. improve a simulation by implementing a new calculation method) create a new branch by checking out the latest main.
- Name your branches: dev-[your_name]-[feature], where "feature" is what you want to achieve (f. ex. "dev-niklas-add-sim-model")
- Use pull-requests to merge your changes: This request should be reviewed by at least the repo owner and one other collaborator
- Communicate using issues: If you have questions or want to add an enhancement, create an issue and discuss everything there. Tag all collaborators who might be interested in this change, but at least one other for a good measure. You can also link issues to pull-requests.
- Push, push, push: Commit and push changes to your branch as often as possible to save your work.

# Commit Tags
To make commit messages more descriptive, please tag them in the following way. Each has to have the form "[TAG1, TAG2, ..] Message":

- [FEATURE]: You added something new.
- [ENHANCEMENT]: You changed existing features that worked already before this commit.
- [FIX]: You fixed a bug.
- [EXP]: Everything experimental which won't bring the project forward. (Hint: Don't push experimental commits to the main, please!)
- [VERSION]: If you publish a stable version. Please also create a release if you use this tag.
We encourage you to invent your own tags if none of these fit your needs.

# Pull requests
Please include the following in your pull request:

- [Description]: What did you change?
- [Reason]: Why did you make these changes?
- [Testing]: How did you verify the changes actually work and don't break anything else?


# foxBMS 2

foxBMS is a free, open and flexible development environment to design battery
management systems. It is the first modular open source BMS development
platform.

## Documentation

A current build of the documentation of this project can be found
here:
- [latest documentation build (of the most recent release)](https://iisb-foxbms.iisb.fraunhofer.de/foxbms/gen2/docs/html/latest/)
- [list of all available documentation builds](https://iisb-foxbms.iisb.fraunhofer.de/foxbms/gen2/docs/html/)

## Changelog

The project changelog is found in
[docs/general/changelog.rst](./docs/general/changelog.rst).

## License

The license information of the project is found in [LICENSE.md](./LICENSE.md).

Basically, the software is covered by the
[**BSD 3-Clause License (SPDX short identifier: BSD-3-Clause)**](https://opensource.org/licenses/BSD-3-Clause)
and the hardware and documentation by the
[**Creative Commons Attribution 4.0 International License (SPDX short identifier: CC-BY-4.0)**](https://creativecommons.org/licenses/by/4.0/legalcode).

## Open Source Hardware Certification

foxBMS 2 has been certified as open source hardware by the
Open Source Hardware Association under the OSHWA UID
[DE000128](https://certification.oshwa.org/de000128.html).

## Acknowledgment

For funding acknowledgements and instructions on how to acknowledge foxBMS 2
please see [foxbms.org/acknowledgements](https://foxbms.org/acknowledgements/).
