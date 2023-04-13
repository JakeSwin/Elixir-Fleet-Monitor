defmodule FleetMonitor.Runtime.Application do
  # See https://hexdocs.pm/elixir/Application.html
  # for more information on OTP Applications
  @moduledoc false
  alias FleetMonitor.Runtime.CameraReader

  use Application

  @impl true
  def start(_type, _args) do
    children = [
      # Starts a worker by calling: FleetMonitor.Worker.start_link(arg)
      # { FleetMonitor.Runtime.CameraReader, [image_topic: "/camera/image_raw"] },
      { FleetMonitor.Runtime.TaskSubmitter, [ ] }
      | generate_camera_readers(FleetMonitor.get_image_topics())
    ]

    # See https://hexdocs.pm/elixir/Supervisor.html
    # for other strategies and supported options
    opts = [strategy: :one_for_one, name: FleetMonitor.Supervisor]

    Supervisor.start_link(children, opts)
  end

  defp generate_camera_readers(nil), do: []
  defp generate_camera_readers(image_topics) do
    Enum.map(image_topics, fn x ->
      Supervisor.child_spec({ FleetMonitor.Runtime.CameraReader, [image_topic: x] },
                            id: {CameraReader, x})
    end)
  end
end
