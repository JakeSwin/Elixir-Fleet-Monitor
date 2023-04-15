defmodule FleetMonitor.Runtime.TaskSubmitter do
  use GenServer
  require Logger

  @python_version "python3.8"
  @python_path Path.join(:code.priv_dir(:fleet_monitor), "/python/task_submitter.py")
  @me __MODULE__

  ### Client Process

  def start_link(args) do
    GenServer.start_link(__MODULE__, args, name: @me)
  end

  def submit_task(start_name, finish_name) do
    GenServer.call(@me, { :submit_task, {start_name, finish_name} })
  end

  ### Server Process
  def init(_) do
    port = Port.open({:spawn, Enum.join([@python_version, @python_path], " ")}, [:binary, :exit_status])
    Port.monitor(port)

    { :ok, %{ port: port, exit_status: nil }}
  end

  def handle_info({ _port, { :data, data }}, state) do
    Logger.info "Info: #{data}"
    # Port.close(port)
    { :noreply, state }
  end

  def handle_info({ _port, { :exit_status, status}}, state) do
    Logger.info "External exit: :exit_status: #{status}"

    { :noreply, %{ state | exit_status: status }}
  end

  def handle_info({:DOWN, _ref, :port, port, :normal}, state) do
    Logger.info "Handled :DOWN message from port: #{inspect port}"
    { :noreply, state }
  end

  def handle_info(_msg, state), do: { :noreply, state }

  def handle_call({ :submit_task, {start_name, finish_name}}, _from, %{port: port} = state) do
    Port.command(port, start_name <> "," <> finish_name <> "\n")
    { :reply, :ok, state }
  end
end
