defmodule FleetMonitor.Runtime.FleetReader do
  use GenServer
  require Logger

  @type t :: pid

  @python_version "python3.8"
  @python_path Path.join(:code.priv_dir(:fleet_monitor), "/python/fleet_state_reader.py")
  @me __MODULE__

  ### Client Process

  def start_link(_) do
    GenServer.start_link(__MODULE__, nil, name: @me)
  end

  ### Server Process
  def init(_) do
    _port = Port.open(
      {:spawn, Enum.join([@python_version, @python_path], " ")},
      [:binary, :exit_status]
    )

    { :ok, %{ exit_status: nil, fleet_state: nil, pids: [] }}
  end

  def handle_info({ _port, { :data, "info: " <> info_data }}, state) do
    Logger.info "Info: #{info_data}"
    { :noreply, state }
  end

  def handle_info({ _port, { :data, data }}, state) do
    if state.fleet_state do
      Phoenix.PubSub.broadcast!(FleetMonitor.PubSub, "fleet_state", {:fleet_state, state.fleet_state})
    end
    fleet_state = case Jason.decode(String.replace(data, "'", "\"")) do
      {:ok, decoded} -> decoded
      {:error, reason} -> IO.puts(reason)
    end
    { :noreply, %{ state | fleet_state: fleet_state }}
  end

  def handle_info({ _port, { :exit_status, status}}, state) do
    Logger.info "External exit: :exit_status: #{status}"

    { :noreply, %{ state | exit_status: status }}
  end

  def handle_info(_msg, state), do: { :noreply, state }
end
