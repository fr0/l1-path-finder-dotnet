# l1-path-finder-dotnet

This is a port of the [l1-path-finder](https://github.com/mikolalysenko/l1-path-finder) project to C# (.NET Standard 2.0).

It depends on the [NumSharp](https://github.com/SciSharp/NumSharp) package.

It was ported directly from the JavaScript code, which is why a lot of the identifier names follow JavaScript
naming conventions (e.g. `lowerCamelCase`) instead of C# naming conventions (e.g. `CamelCase`).

## TODO

* Fix up the API to use C# naming conventions (pull requests welcome)
* Make more fields `private` when possible (pull requests welcome)
* Port the unit tests (pull requests welcome)
* Publish to NuGet
