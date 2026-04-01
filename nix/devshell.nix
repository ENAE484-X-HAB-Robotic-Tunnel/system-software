{
    pkgs,
    perSystem,
    ...
}:
perSystem.devshell.mkShell {
    name = "ENAE484 X-HAB Robotic Tunnel: system-software module";
    motd = ''
        {141}ENAE484 X-HAB Robotic Tunnel:{reset} {51}system-software module{reset} devshell
        $(type -p menu &>/dev/null && menu)
    '';

    commands = [
        {
            category = "[development]";
            package = pkgs.ruff;
        }
        {
            category = "[development]";
            package = pkgs.uv;
        }

        # TODO:
        # choose and set up build-system/packaging & documentation tools
        # {
        #     category = "[packaging]";
        #     package = pkgs.colcon;
        # }
        # {
        #     category = "[packaging]";
        #     package = pkgs.mkdocs;
        # }
        {
            category = "[packaging]";
            name = "ghmd";
            help = "GitHub-style preview for markdown file(s)";
            command = "gh-markdown-preview \$@";
        }
    ];

    packages = with pkgs; [
        gh-markdown-preview
        python3
    ];

    env = [ ];
}
