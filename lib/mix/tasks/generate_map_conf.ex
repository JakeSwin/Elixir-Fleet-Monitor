defmodule Mix.Tasks.GenerateMapConf do
  use Mix.Task

  @shortdoc "Generates current map config file"

  @python_version "python3.8"
  @python_path Path.join(:code.priv_dir(:fleet_monitor), "/python/read_building_map.py")

  def run(_) do
    _port = Port.open({
      :spawn, Enum.join([@python_version, @python_path], " ")
    }, [:binary])

    receive do
      {_port, {:data, data}} ->
        IO.puts(data)
      _ ->
        IO.puts(:stderr, "Unexpected message, task cancelled")
    after
      10000 ->
        IO.puts(:stderr, "No message in 10 seconds, task cancelled")
    end
  end
end
