defmodule Mix.Tasks.GenerateMapConf do
  use Mix.Task

  @shortdoc "Generates current map config file"

  @command "python3.8 " <> Path.expand("./lib/python/read_building_map.py")

  def run(_) do
    _port = Port.open({:spawn, @command}, [:binary])
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
